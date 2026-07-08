#!/usr/bin/env bash
set -euo pipefail

# Run on 172.16.0.250 after conda is available.
# The default target is a local 7B/8B Qwen-VL runtime suitable for a single RTX 4090.

conda create -y -n qwen-vla python=3.10
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate qwen-vla
pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install 'transformers>=4.57.0' accelerate qwen-vl-utils pillow
