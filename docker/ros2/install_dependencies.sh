#!/usr/bin/env bash
# Run inside ros2-dev; uses /usr/bin/python3 directly (no Conda/venv).
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="${PROJECT_DIR:-/workspace/SocialEmbodiedAgent}"
mode="${1:-full}"
if [[ "$mode" != full && "$mode" != --dependencies-only ]]; then
    printf 'Usage: bash install_dependencies.sh [--dependencies-only]\n' >&2
    exit 2
fi

/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), sys.version'
if [[ "$mode" == full ]]; then
    test -f "$project_dir/visualnav-transformer/train/setup.py"
    test -f "$project_dir/diffusion_policy/setup.py"
fi

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-pip python3-dev build-essential git ffmpeg libgl1 libglib2.0-0 \
    python3-opencv python3-cryptography openssl ros-humble-cv-bridge ros-humble-image-transport

# CUDA runtime libraries come with the PyTorch wheels; nvcc is not required
# by this inference workflow. The NVIDIA driver is provided by the host.
if [[ -n "${NOMAD_WHEELHOUSE:-}" ]]; then
    # A plain --find-links can still prefer an equivalent remote wheel.
    # Install the verified GPU wheels offline, then resolve small dependencies
    # in the requirements pass below.
    /usr/bin/python3 -m pip install --no-cache-dir --progress-bar off \
        --no-index --no-deps -c "$script_dir/requirements.txt" \
        "$NOMAD_WHEELHOUSE"/*.whl
else
    /usr/bin/python3 -m pip install --no-cache-dir --progress-bar off \
        -c "$script_dir/requirements.txt" \
        torch==2.6.0+cu124 torchvision==0.21.0+cu124 \
        --index-url https://download.pytorch.org/whl/cu124
fi
/usr/bin/python3 -m pip install --no-cache-dir --progress-bar off \
    -r "$script_dir/requirements.txt"
if [[ "$mode" == full ]]; then
    /usr/bin/python3 -m pip install --no-deps --no-build-isolation \
        -e "$project_dir/visualnav-transformer/train" \
        -e "$project_dir/diffusion_policy"
fi

source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pip check
apt-get clean
printf 'Dependencies installed. Validate with docker/ros2/verify_environment.py.\n'
