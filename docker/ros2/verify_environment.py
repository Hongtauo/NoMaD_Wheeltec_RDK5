"""Check Humble, system Python, CUDA, image conversion and ROS demo messaging."""

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-root", type=Path,
        default=Path("/workspace/SocialEmbodiedAgent/outputs/environment_checks"),
    )
    args = parser.parse_args()
    run_dir = args.output_root / datetime.now(timezone.utc).strftime("ros2-dev__%Y%m%dT%H%M%S.%fZ")
    run_dir.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "python": sys.version, "executable": sys.executable}
    try:
        assert sys.executable == "/usr/bin/python3", sys.executable
        assert sys.version_info[:2] == (3, 10), sys.version
        assert os.environ.get("ROS_DISTRO") == "humble", "Source Humble setup.bash first"

        import cv2
        from cv_bridge import CvBridge
        import numpy as np
        import rclpy
        import torch
        import torchvision
        import diffusers
        import vint_train
        import scipy.linalg
        from mpl_toolkits.mplot3d import Axes3D
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
        from vint_train.training.train_utils import get_action

        assert torch.cuda.is_available(), "CUDA unavailable"
        tensor = torch.arange(16, dtype=torch.float32, device="cuda").reshape(4, 4)
        torch.testing.assert_close((tensor @ tensor.T).cpu(), tensor.cpu() @ tensor.cpu().T)
        # Exercise torchvision's compiled CUDA extension as well as torch.
        keep = torchvision.ops.nms(
            torch.tensor([[0, 0, 1, 1], [0, 0, 1, 1]], dtype=torch.float32, device="cuda"),
            torch.tensor([0.9, 0.8], device="cuda"), 0.5,
        )
        assert keep.tolist() == [0]

        bridge = CvBridge()
        rgb = np.arange(12 * 16 * 3, dtype=np.uint8).reshape(12, 16, 3)
        message = bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        bgr = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        np.testing.assert_array_equal(bgr, rgb[:, :, ::-1])
        np.testing.assert_allclose(scipy.linalg.solve(np.eye(2), [1., 2.]), [1., 2.])
        figure = Figure(figsize=(3, 3))
        canvas = FigureCanvasAgg(figure)
        axis = figure.add_subplot(projection="3d")
        axis.plot([0, 1], [0, 1], [0, 1])
        canvas.print_png(run_dir / "plot-smoke.png")
        node = None
        rclpy.init()
        try:
            node = rclpy.create_node("nomad_environment_check")
            report["rclpy_node"] = node.get_name()
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

        report.update({
            "ros_distro": os.environ["ROS_DISTRO"],
            "gpu": torch.cuda.get_device_name(0),
            "cuda_runtime": torch.version.cuda,
            "opencv": cv2.__version__,
            "checks": ["cuda_matmul", "torchvision_cuda_nms", "cv_bridge_rgb_to_bgr", "scipy_solve", "matplotlib_3d_render", "rclpy_node"],
            "packages": {name: importlib.metadata.version(name) for name in (
                "torch", "torchvision", "numpy", "matplotlib", "Pillow", "diffusers",
                "huggingface-hub", "efficientnet-pytorch", "einops", "wandb", "setuptools", "scipy",
            )},
        })

        # Isolate the smoke test from robot domains. These programs only publish /chatter.
        ros_env = {**os.environ, "ROS_DOMAIN_ID": "173", "ROS_LOCALHOST_ONLY": "1"}
        processes = []
        with (run_dir / "listener.log").open("w") as listener_log, (run_dir / "talker.log").open("w") as talker_log:
            try:
                for name, output in (("listener", listener_log), ("talker", talker_log)):
                    processes.append(subprocess.Popen(
                        [f"/opt/ros/humble/lib/demo_nodes_cpp/{name}"],
                        stdout=output, stderr=subprocess.STDOUT, env=ros_env,
                    ))
                time.sleep(7)
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                for process in processes:
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
        listener_text = (run_dir / "listener.log").read_text()
        received = listener_text.count("I heard:")
        assert received >= 2, listener_text
        assert all(process.returncode == 0 for process in processes)
        report["talker_listener_received"] = received
        report["checks"].append("talker_listener")
        report["pip_check"] = subprocess.check_output(
            [sys.executable, "-m", "pip", "check"], text=True,
        ).strip()
        (run_dir / "pip-freeze.txt").write_text(subprocess.check_output(
            [sys.executable, "-m", "pip", "freeze", "--all"], text=True,
        ))
        (run_dir / "system-packages.txt").write_text(subprocess.check_output(
            ["dpkg-query", "-W"], text=True,
        ))
        (run_dir / "gpu.txt").write_text(subprocess.check_output(["nvidia-smi"], text=True))
        report["status"] = "complete"
    except Exception as error:
        report.update(status="failed", error=repr(error))
        raise
    finally:
        (run_dir / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        print(f"Report directory: {run_dir}")


if __name__ == "__main__":
    main()
