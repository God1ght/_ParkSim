#!/usr/bin/env bash
set -euo pipefail

# Run on 172.16.0.250 after conda is available.
# The default target is a local 7B/8B Qwen-VL runtime suitable for a single RTX 4090.
# TUNA mirrors may expose metadata while returning 403 for package artifacts on this host,
# so the script pins conda/PyPI package downloads to official endpoints that were verified here.

source "$(conda info --base)/etc/profile.d/conda.sh"
if ! conda env list | awk '{print $1}' | grep -qx qwen-vla; then
  conda create -y -n qwen-vla python=3.10 \
    --override-channels \
    -c https://repo.anaconda.com/pkgs/main \
    -c https://repo.anaconda.com/pkgs/r
fi
conda activate qwen-vla
pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install -i https://pypi.org/simple 'transformers>=4.57.0,<5' accelerate qwen-vl-utils pillow
