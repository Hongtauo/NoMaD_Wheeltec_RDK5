"""Convert a video into an ordered RGB frame sequence for NoMaD inference."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return number


def nonnegative_float(value: str) -> float:
    number = float(value)
    if number < 0:
        raise argparse.ArgumentTypeError("value must be zero or greater")
    return number


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Extract an MP4 (or other FFmpeg-supported video) into ordered JPG "
            "frames suitable for NoMaD offline inference."
        )
    )
    parser.add_argument("video", type=Path, help="input video path")
    parser.add_argument(
        "--output-dir",
        "-o",
        type=Path,
        help="output directory (default: <video_stem>_frames next to the video)",
    )
    parser.add_argument(
        "--fps",
        type=positive_float,
        default=4.0,
        help="sampling rate in frames per second (default: 4)",
    )
    parser.add_argument(
        "--start",
        type=nonnegative_float,
        default=0.0,
        help="start time in seconds (default: 0)",
    )
    parser.add_argument(
        "--duration",
        type=positive_float,
        help="optional segment duration in seconds",
    )
    parser.add_argument(
        "--quality",
        type=int,
        choices=range(2, 32),
        default=2,
        metavar="2-31",
        help="JPEG quality: 2 is highest and 31 is lowest (default: 2)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace JPG frames previously generated in the output directory",
    )
    return parser.parse_args()


def require_program(name: str) -> str:
    executable = shutil.which(name)
    if executable is None:
        raise RuntimeError(
            f"{name} was not found. Install FFmpeg before running this script."
        )
    return executable


def probe_video(ffprobe: str, video: Path) -> dict:
    command = [
        ffprobe,
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height,avg_frame_rate,duration:format=duration",
        "-of",
        "json",
        str(video),
    ]
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    return json.loads(result.stdout)


def main() -> int:
    args = parse_args()
    video = args.video.expanduser().resolve()
    if not video.is_file():
        raise FileNotFoundError(f"input video does not exist: {video}")

    ffmpeg = require_program("ffmpeg")
    ffprobe = require_program("ffprobe")
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir
        else video.with_name(f"{video.stem}_frames")
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    existing_frames = sorted(output_dir.glob("*.jpg"))
    if existing_frames and not args.overwrite:
        raise FileExistsError(
            f"{output_dir} already contains {len(existing_frames)} JPG files; "
            "choose another directory or pass --overwrite"
        )
    if args.overwrite:
        for frame in existing_frames:
            frame.unlink()

    probe = probe_video(ffprobe, video)
    output_pattern = output_dir / "%06d.jpg"
    command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y"]
    if args.start:
        command.extend(["-ss", str(args.start)])
    command.extend(["-i", str(video)])
    if args.duration is not None:
        command.extend(["-t", str(args.duration)])
    command.extend(
        [
            "-map",
            "0:v:0",
            "-vf",
            f"fps={args.fps}",
            "-q:v",
            str(args.quality),
            "-start_number",
            "0",
            str(output_pattern),
        ]
    )

    print(f"Input:  {video}")
    print(f"Output: {output_dir}")
    print(f"Sampling at {args.fps:g} FPS ...")
    subprocess.run(command, check=True)

    frames = sorted(output_dir.glob("*.jpg"))
    if len(frames) < 4:
        raise RuntimeError(
            f"only {len(frames)} frames were extracted; NoMaD requires at least 4"
        )

    metadata = {
        "source_video": str(video),
        "output_directory": str(output_dir),
        "sampling_fps": args.fps,
        "start_seconds": args.start,
        "duration_seconds": args.duration,
        "jpeg_quality": args.quality,
        "frame_count": len(frames),
        "first_frame": frames[0].name,
        "last_frame": frames[-1].name,
        "video_probe": probe,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    metadata_path = output_dir / "metadata.json"
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    print(f"Extracted {len(frames)} frames.")
    print(f"Metadata: {metadata_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, FileExistsError, RuntimeError) as error:
        print(f"Error: {error}", file=sys.stderr)
        raise SystemExit(1)
    except subprocess.CalledProcessError as error:
        print(f"FFmpeg failed with exit code {error.returncode}", file=sys.stderr)
        raise SystemExit(error.returncode)
