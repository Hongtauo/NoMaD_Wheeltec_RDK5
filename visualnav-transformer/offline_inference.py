"""Run goal-free NoMaD inference on an ordered directory of RGB frames."""

from __future__ import annotations

import argparse
import json
import math
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
import yaml
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
from efficientnet_pytorch import EfficientNet  # noqa: F401 - validates dependency
from PIL import Image, ImageOps
from torchvision import transforms

from vint_train.models.nomad.nomad import DenseNetwork, NoMaD
from vint_train.models.nomad.nomad_vint import NoMaD_ViNT, replace_bn_with_gn


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR / "train" / "config" / "nomad.yaml"
DEFAULT_CHECKPOINT = SCRIPT_DIR / "deployment" / "model_weights" / "nomad.pth"
DEFAULT_OUTPUT_ROOT = SCRIPT_DIR.parent / "offline_outputs"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run NoMaD goal-free diffusion inference without ROS."
    )
    parser.add_argument("image_dir", type=Path, help="ordered RGB frame directory")
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument(
        "--run-name",
        help="optional run-name prefix; defaults to the input directory name",
    )
    parser.add_argument("--num-samples", type=int, default=8)
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="advance this many source frames between inference windows (default: 1)",
    )
    parser.add_argument(
        "--max-windows",
        type=int,
        help="optional maximum number of inference windows for a quick test",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--device",
        default="cuda",
        help="PyTorch device (default: cuda)",
    )
    return parser.parse_args()


def require_positive(name: str, value: int) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be greater than zero")


def natural_key(path: Path) -> list[Any]:
    return [int(token) if token.isdigit() else token.lower() for token in re.split(r"(\d+)", path.name)]


def list_frames(image_dir: Path) -> list[Path]:
    frames = sorted(
        (path for path in image_dir.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES),
        key=natural_key,
    )
    return frames


def slugify(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9._-]+", "-", value.strip())
    value = re.sub(r"-+", "-", value).strip("-._")
    return value or "sequence"


def create_run_directory(output_root: Path, run_prefix: str) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    base_name = f"{slugify(run_prefix)}__nomad-goalfree__{timestamp}"
    for suffix in range(1000):
        name = base_name if suffix == 0 else f"{base_name}__{suffix:02d}"
        candidate = output_root / name
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError("could not allocate a unique output directory")


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def build_model(config: dict[str, Any], checkpoint: Path, device: torch.device) -> NoMaD:
    if config.get("model_type") != "nomad":
        raise ValueError(f"expected model_type=nomad, got {config.get('model_type')!r}")
    if config.get("vision_encoder") != "nomad_vint":
        raise ValueError("this offline entry point currently supports vision_encoder=nomad_vint")

    vision_encoder = NoMaD_ViNT(
        obs_encoding_size=config["encoding_size"],
        context_size=config["context_size"],
        mha_num_attention_heads=config["mha_num_attention_heads"],
        mha_num_attention_layers=config["mha_num_attention_layers"],
        mha_ff_dim_factor=config["mha_ff_dim_factor"],
    )
    vision_encoder = replace_bn_with_gn(vision_encoder)
    noise_pred_net = ConditionalUnet1D(
        input_dim=2,
        global_cond_dim=config["encoding_size"],
        down_dims=config["down_dims"],
        cond_predict_scale=config["cond_predict_scale"],
    )
    model = NoMaD(
        vision_encoder=vision_encoder,
        noise_pred_net=noise_pred_net,
        dist_pred_net=DenseNetwork(embedding_dim=config["encoding_size"]),
    )

    state_dict = torch.load(checkpoint, map_location="cpu", weights_only=True)
    incompatible = model.load_state_dict(state_dict, strict=False)
    if incompatible.missing_keys or incompatible.unexpected_keys:
        raise RuntimeError(
            "checkpoint does not exactly match the configured model: "
            f"missing={incompatible.missing_keys}, unexpected={incompatible.unexpected_keys}"
        )
    return model.to(device).eval()


def load_observation(frame_paths: list[Path], image_size: list[int]) -> torch.Tensor:
    transform = transforms.Compose(
        [
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225],
            ),
        ]
    )
    images = []
    for path in frame_paths:
        with Image.open(path) as source:
            image = ImageOps.exif_transpose(source).convert("RGB")
            image = image.resize(tuple(image_size), Image.Resampling.BILINEAR)
            images.append(transform(image).unsqueeze(0))
    return torch.cat(images, dim=1)


def decode_actions(diffusion_output: torch.Tensor, action_stats: dict[str, Any]) -> np.ndarray:
    normalized_deltas = diffusion_output.detach().cpu().numpy()
    minimum = np.asarray(action_stats["min"], dtype=np.float32)
    maximum = np.asarray(action_stats["max"], dtype=np.float32)
    deltas = (normalized_deltas + 1.0) * 0.5
    deltas = deltas * (maximum - minimum) + minimum
    return np.cumsum(deltas, axis=1)


def infer_window(
    model: NoMaD,
    scheduler: DDPMScheduler,
    observation: torch.Tensor,
    config: dict[str, Any],
    num_samples: int,
    device: torch.device,
    seed: int,
) -> tuple[np.ndarray, float]:
    generator = torch.Generator(device=device).manual_seed(seed)
    observation = observation.to(device)
    fake_goal = torch.randn(
        (1, 3, *config["image_size"]), device=device, generator=generator
    )
    goal_mask = torch.ones(1, dtype=torch.long, device=device)

    if device.type == "cuda":
        torch.cuda.synchronize(device)
    start = time.perf_counter()
    with torch.inference_mode():
        condition = model(
            "vision_encoder",
            obs_img=observation,
            goal_img=fake_goal,
            input_goal_mask=goal_mask,
        )
        condition = condition.repeat(num_samples, 1)
        action = torch.randn(
            (num_samples, config["len_traj_pred"], 2),
            device=device,
            generator=generator,
        )
        scheduler.set_timesteps(config["num_diffusion_iters"])
        for timestep in scheduler.timesteps:
            noise_prediction = model(
                "noise_pred_net",
                sample=action,
                timestep=timestep.to(device),
                global_cond=condition,
            )
            action = scheduler.step(
                model_output=noise_prediction,
                timestep=timestep,
                sample=action,
            ).prev_sample
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    elapsed = time.perf_counter() - start
    return action, elapsed


def draw_trajectory_axis(axis: plt.Axes, trajectories: np.ndarray) -> None:
    colors = plt.cm.tab10(np.linspace(0, 1, trajectories.shape[0]))
    for index, (trajectory, color) in enumerate(zip(trajectories, colors)):
        forward = np.concatenate(([0.0], trajectory[:, 0]))
        lateral = np.concatenate(([0.0], trajectory[:, 1]))
        axis.plot(lateral, forward, marker="o", markersize=2.5, linewidth=1.4, color=color, label=f"T{index + 1}")
    axis.scatter([0], [0], marker="^", s=80, color="black", zorder=5, label="robot")
    axis.axhline(0, color="0.75", linewidth=0.8)
    axis.axvline(0, color="0.75", linewidth=0.8)
    axis.set_xlabel("lateral y")
    axis.set_ylabel("forward x")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.25)


def save_window_plot(
    output: Path,
    current_frame: Path,
    trajectories: np.ndarray,
    title: str,
) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    with Image.open(current_frame) as source:
        axes[0].imshow(ImageOps.exif_transpose(source).convert("RGB"))
    axes[0].set_title(current_frame.name)
    axes[0].axis("off")
    draw_trajectory_axis(axes[1], trajectories)
    axes[1].set_title("8 sampled trajectories")
    figure.suptitle(title)
    figure.savefig(output, dpi=140)
    plt.close(figure)


def save_summary_plot(
    output: Path,
    frames: list[Path],
    end_indices: np.ndarray,
    trajectories: np.ndarray,
) -> None:
    count = min(6, len(end_indices))
    selected = np.unique(np.linspace(0, len(end_indices) - 1, count, dtype=int))
    figure, axes = plt.subplots(len(selected), 2, figsize=(12, 4 * len(selected)), constrained_layout=True)
    axes = np.atleast_2d(axes)
    for row, result_index in enumerate(selected):
        frame = frames[int(end_indices[result_index])]
        with Image.open(frame) as source:
            axes[row, 0].imshow(ImageOps.exif_transpose(source).convert("RGB"))
        axes[row, 0].set_title(f"window {result_index:04d} · {frame.name}")
        axes[row, 0].axis("off")
        draw_trajectory_axis(axes[row, 1], trajectories[result_index])
        axes[row, 1].set_title("goal-free candidate trajectories")
    figure.savefig(output, dpi=140)
    plt.close(figure)


def read_sampling_fps(image_dir: Path) -> float | None:
    metadata_path = image_dir / "metadata.json"
    if not metadata_path.is_file():
        return None
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        return float(metadata["sampling_fps"])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def json_ready_path(path: Path) -> str:
    return str(path.resolve())


def main() -> int:
    args = parse_args()
    require_positive("num_samples", args.num_samples)
    require_positive("stride", args.stride)
    if args.max_windows is not None:
        require_positive("max_windows", args.max_windows)

    image_dir = args.image_dir.expanduser().resolve()
    checkpoint = args.checkpoint.expanduser().resolve()
    config_path = args.config.expanduser().resolve()
    if not image_dir.is_dir():
        raise FileNotFoundError(f"image directory not found: {image_dir}")
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")
    if not config_path.is_file():
        raise FileNotFoundError(f"config not found: {config_path}")

    config = load_yaml(config_path)
    context_frames = int(config["context_size"]) + 1
    frames = list_frames(image_dir)
    if len(frames) < context_frames:
        raise ValueError(
            f"found {len(frames)} frames, but NoMaD requires at least {context_frames}"
        )

    end_indices = list(range(context_frames - 1, len(frames), args.stride))
    if args.max_windows is not None:
        end_indices = end_indices[: args.max_windows]
    run_prefix = args.run_name or image_dir.name.removesuffix("_frames")
    run_dir = create_run_directory(args.output_root.expanduser().resolve(), run_prefix)
    plots_dir = run_dir / "plots"
    plots_dir.mkdir()

    requested_device = torch.device(args.device)
    if requested_device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")
    device = requested_device
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    manifest = {
        "status": "running",
        "run_name": run_dir.name,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "input": {
            "image_dir": json_ready_path(image_dir),
            "frame_count": len(frames),
            "sampling_fps": read_sampling_fps(image_dir),
        },
        "model": {
            "name": "NoMaD",
            "mode": "goal-free",
            "checkpoint": json_ready_path(checkpoint),
            "config": json_ready_path(config_path),
        },
        "inference": {
            "context_frames": context_frames,
            "num_samples": args.num_samples,
            "trajectory_length": config["len_traj_pred"],
            "diffusion_iterations": config["num_diffusion_iters"],
            "stride": args.stride,
            "window_count": len(end_indices),
            "seed": args.seed,
            "device": str(device),
        },
        "software": {
            "torch": torch.__version__,
            "cuda_runtime": torch.version.cuda,
        },
    }
    manifest_path = run_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print(f"Run directory: {run_dir}")
    print(f"Loading checkpoint: {checkpoint}")
    model = build_model(config, checkpoint, device)
    scheduler = DDPMScheduler(
        num_train_timesteps=config["num_diffusion_iters"],
        beta_schedule="squaredcos_cap_v2",
        clip_sample=True,
        prediction_type="epsilon",
    )
    with (SCRIPT_DIR / "train" / "vint_train" / "data" / "data_config.yaml").open(
        "r", encoding="utf-8"
    ) as file:
        action_stats = yaml.safe_load(file)["action_stats"]

    all_trajectories = []
    timings = []
    for window_index, end_index in enumerate(end_indices):
        start_index = end_index - config["context_size"]
        observation = load_observation(
            frames[start_index : end_index + 1], config["image_size"]
        )
        diffusion_output, elapsed = infer_window(
            model=model,
            scheduler=scheduler,
            observation=observation,
            config=config,
            num_samples=args.num_samples,
            device=device,
            seed=args.seed + window_index,
        )
        trajectories = decode_actions(diffusion_output, action_stats)
        all_trajectories.append(trajectories)
        timings.append(elapsed)

        sampling_fps = manifest["input"]["sampling_fps"]
        time_ms = round(1000 * end_index / sampling_fps) if sampling_fps else end_index
        plot_name = (
            f"window_{window_index:04d}__end_{end_index:06d}__t{time_ms:07d}ms.png"
        )
        save_window_plot(
            plots_dir / plot_name,
            frames[end_index],
            trajectories,
            f"NoMaD goal-free · {elapsed * 1000:.1f} ms",
        )
        print(
            f"[{window_index + 1:04d}/{len(end_indices):04d}] "
            f"{frames[end_index].name} · {elapsed * 1000:.1f} ms"
        )

    trajectory_array = np.stack(all_trajectories)
    end_index_array = np.asarray(end_indices, dtype=np.int32)
    timing_array = np.asarray(timings, dtype=np.float32)
    np.savez_compressed(
        run_dir / "trajectories.npz",
        trajectories=trajectory_array,
        end_indices=end_index_array,
        end_frame_names=np.asarray([frames[index].name for index in end_indices]),
        inference_seconds=timing_array,
    )
    save_summary_plot(
        run_dir / "summary.png", frames, end_index_array, trajectory_array
    )

    manifest["status"] = "complete"
    manifest["completed_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["performance"] = {
        "mean_inference_ms": float(timing_array.mean() * 1000),
        "median_inference_ms": float(np.median(timing_array) * 1000),
        "min_inference_ms": float(timing_array.min() * 1000),
        "max_inference_ms": float(timing_array.max() * 1000),
        "peak_cuda_memory_mib": (
            float(torch.cuda.max_memory_allocated(device) / 1024**2)
            if device.type == "cuda"
            else None
        ),
    }
    manifest["outputs"] = {
        "trajectory_archive": "trajectories.npz",
        "summary_plot": "summary.png",
        "plot_directory": "plots",
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Completed: {run_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        print(f"Error: {error}")
        raise SystemExit(1)
