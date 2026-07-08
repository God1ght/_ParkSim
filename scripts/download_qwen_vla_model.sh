#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODEL_ID="${QWEN_MODEL_ID:-Qwen/Qwen2.5-VL-7B-Instruct}"
CACHE_DIR="${QWEN_MODEL_CACHE:-/media/step/data/models/huggingface}"
ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"

source /home/step/anaconda3/etc/profile.d/conda.sh
conda activate qwen-vla

mkdir -p "$CACHE_DIR"
export HF_HOME="$CACHE_DIR"
export HUGGINGFACE_HUB_CACHE="$CACHE_DIR/hub"
export HF_ENDPOINT="$ENDPOINT"

python - "$MODEL_ID" "$CACHE_DIR" <<'PYCODE'
import sys
from huggingface_hub import snapshot_download

model_id = sys.argv[1]
cache_dir = sys.argv[2]
path = snapshot_download(
    repo_id=model_id,
    cache_dir=cache_dir,
    resume_download=True,
    local_files_only=False,
)
print(path)
PYCODE

echo "Downloaded $MODEL_ID into $CACHE_DIR via $ENDPOINT"
echo "Use: QWEN_MODEL_CACHE=$CACHE_DIR QWEN_MODEL_ID=$MODEL_ID $ROOT/scripts/run_qwen_vla_service.sh"
