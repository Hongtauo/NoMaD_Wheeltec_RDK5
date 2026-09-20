"""ROS-independent observation guards, action decoding and goal-free NoMaD."""

from collections import deque
import math
from pathlib import Path
import time

import numpy as np
import yaml


class ObservationBuffer:
    """Keep only fresh, ordered, approximately equally spaced RGB observations."""

    def __init__(self, count, hz, timeout, max_gap, future_tolerance=0.2):
        self.frames = deque(maxlen=count)
        self.min_spacing_ns = int(0.9 * 1e9 / hz)
        self.timeout_ns = int(timeout * 1e9)
        self.max_gap_ns = int(max_gap * 1e9)
        self.future_ns = int(future_tolerance * 1e9)
        self.last_arrival = None
        self.last_seen_stamp = None

    def clear(self):
        self.frames.clear()
        self.last_arrival = None
        self.last_seen_stamp = None

    def add(self, rgb, stamp_ns, frame_id, now_ns, arrival):
        if not frame_id or stamp_ns <= 0 or now_ns - stamp_ns > self.timeout_ns or stamp_ns - now_ns > self.future_ns:
            self.clear()
            return False, 'invalid_or_stale_stamp'
        reset = None
        if self.frames and (
            stamp_ns <= self.last_seen_stamp
            or stamp_ns - self.frames[-1][0] > self.max_gap_ns
            or frame_id != self.frames[-1][2]
            or arrival - self.last_arrival > self.timeout_ns / 1e9
        ):
            self.clear()
            reset = 'observation_discontinuity'
        self.last_seen_stamp = stamp_ns
        if self.frames and stamp_ns - self.frames[-1][0] < self.min_spacing_ns:
            return False, None
        self.frames.append((stamp_ns, rgb.copy(), frame_id))
        self.last_arrival = arrival
        return True, reset

    def fresh(self, now_ns, monotonic):
        if not self.frames:
            return False
        age = now_ns - self.frames[-1][0]
        return (-self.future_ns <= age <= self.timeout_ns
                and monotonic - self.last_arrival <= self.timeout_ns / 1e9)

    @property
    def ready(self):
        return len(self.frames) == self.frames.maxlen


def select_waypoint(actions, index, scale, samples, steps):
    actions = np.asarray(actions, dtype=np.float32)
    if actions.shape != (samples, steps, 2) or not np.isfinite(actions).all():
        raise ValueError('Model returned nonfinite or incorrectly shaped trajectories')
    if not 0 <= index < steps or not math.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid waypoint index or scale')
    waypoint = actions[0, index] * scale
    if not np.isfinite(waypoint).all():
        raise ValueError('Scaled waypoint is not finite')
    return waypoint


def preprocess_rgb(frames, image_size):
    """Match official deployment transform_images(..., center_crop=False)."""
    import torch
    from PIL import Image
    from torchvision import transforms

    normalize = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])
    return torch.cat([
        normalize(Image.fromarray(frame).resize(tuple(image_size), Image.Resampling.BICUBIC)).unsqueeze(0)
        for frame in frames
    ], dim=1)


class NoMaDBackend:
    """Load local trusted weights; no downloads, ROS 1 imports or path injection."""

    def __init__(self, params):
        import torch
        from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
        from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
        from vint_train.models.nomad.nomad import NoMaD, DenseNetwork
        from vint_train.models.nomad.nomad_vint import NoMaD_ViNT, replace_bn_with_gn

        self.torch = torch
        for key in ('model_config', 'checkpoint', 'action_stats_path'):
            if not Path(params[key]).is_file():
                raise FileNotFoundError(f'{key} file not found: {params[key]}')
        self.config = yaml.safe_load(Path(params['model_config']).read_text(encoding='utf-8'))
        config = self.config
        if config.get('model_type') != 'nomad' or config.get('vision_encoder') != 'nomad_vint':
            raise ValueError('This node supports NoMaD with the official nomad_vint encoder only')
        if config['image_size'] != [96, 96] or config['context_size'] < 0:
            raise ValueError('Expected the official 96x96 NoMaD configuration')
        if config['len_traj_pred'] <= 0 or config['num_diffusion_iters'] <= 0:
            raise ValueError('Invalid model trajectory or diffusion configuration')
        self.context_frames = config['context_size'] + 1
        self.device = torch.device(params['device'])
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA requested but unavailable; no silent CPU fallback')
        if self.device.type not in ('cpu', 'cuda'):
            raise ValueError('device must be cpu or cuda')
        self.num_samples = params['num_samples']
        self.seed = params['seed']
        stats = yaml.safe_load(Path(params['action_stats_path']).read_text(encoding='utf-8'))['action_stats']
        # Official get_action uses float64 statistics before casting back to float32.
        self.minimum = np.asarray(stats['min'], dtype=np.float64)
        self.maximum = np.asarray(stats['max'], dtype=np.float64)
        if (self.minimum.shape != (2,) or self.maximum.shape != (2,)
                or not np.isfinite([self.minimum, self.maximum]).all()
                or not np.all(self.maximum > self.minimum)):
            raise ValueError('Invalid action normalization statistics')
        encoder = replace_bn_with_gn(NoMaD_ViNT(
            obs_encoding_size=config['encoding_size'], context_size=config['context_size'],
            mha_num_attention_heads=config['mha_num_attention_heads'],
            mha_num_attention_layers=config['mha_num_attention_layers'],
            mha_ff_dim_factor=config['mha_ff_dim_factor']))
        self.model = NoMaD(
            vision_encoder=encoder,
            noise_pred_net=ConditionalUnet1D(
                input_dim=2, global_cond_dim=config['encoding_size'],
                down_dims=config['down_dims'], cond_predict_scale=config['cond_predict_scale']),
            dist_pred_net=DenseNetwork(embedding_dim=config['encoding_size']))
        weights = torch.load(params['checkpoint'], map_location='cpu', weights_only=True)
        self.model.load_state_dict(weights, strict=True)
        self.model.to(self.device).eval()
        self.runtime_info = {
            'torch': str(torch.__version__), 'device': str(next(self.model.parameters()).device),
            'cuda_runtime': torch.version.cuda,
            'gpu': torch.cuda.get_device_name(self.device) if self.device.type == 'cuda' else None,
        }
        self.scheduler = DDPMScheduler(
            num_train_timesteps=config['num_diffusion_iters'], beta_schedule='squaredcos_cap_v2',
            clip_sample=True, prediction_type='epsilon')
        self.warmup_ms = None
        if params['warmup']:
            blank = np.zeros((96, 96, 3), dtype=np.uint8)
            _, self.warmup_ms = self.infer([blank] * self.context_frames, 0)

    def decode(self, diffusion_output):
        deltas = (diffusion_output.detach().cpu().numpy() + 1.0) / 2.0
        deltas = deltas * (self.maximum - self.minimum) + self.minimum
        return np.cumsum(deltas, axis=1).astype(np.float32)

    def infer(self, frames, sequence):
        torch = self.torch
        if len(frames) != self.context_frames:
            raise ValueError('Incomplete observation context')
        started = time.perf_counter()
        observation = preprocess_rgb(frames, self.config['image_size']).to(self.device)
        generator = torch.Generator(device=self.device).manual_seed(self.seed + sequence)
        with torch.inference_mode():
            goal = torch.randn((1, 3, 96, 96), device=self.device, generator=generator)
            mask = torch.ones(1, dtype=torch.long, device=self.device)
            condition = self.model('vision_encoder', obs_img=observation,
                                   goal_img=goal, input_goal_mask=mask).repeat(self.num_samples, 1)
            action = torch.randn((self.num_samples, self.config['len_traj_pred'], 2),
                                 device=self.device, generator=generator)
            self.scheduler.set_timesteps(self.config['num_diffusion_iters'])
            for step in self.scheduler.timesteps:
                prediction = self.model('noise_pred_net', sample=action,
                                        timestep=step.to(self.device), global_cond=condition)
                action = self.scheduler.step(prediction, step, action, generator=generator).prev_sample
            result = self.decode(action)
        return result, (time.perf_counter() - started) * 1000
