#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HOST="${QWEN_VLA_HOST:-127.0.0.1}"
PORT="${QWEN_VLA_PORT:-8000}"
MODEL_ID="${QWEN_MODEL_ID:-Qwen/Qwen2.5-VL-7B-Instruct}"
CACHE_DIR="${QWEN_MODEL_CACHE:-/media/step/data/models/huggingface}"
LOCAL_SNAPSHOT="${QWEN_MODEL_LOCAL_SNAPSHOT:-}"
LOCAL_MODE="${QWEN_MODEL_LOCAL_ONLY:-auto}"
DTYPE="${QWEN_TORCH_DTYPE:-auto}"
DEVICE_MAP="${QWEN_DEVICE_MAP:-auto}"
MAX_NEW_TOKENS="${QWEN_MAX_NEW_TOKENS:-128}"
ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"

if [[ -z "$LOCAL_SNAPSHOT" && "$LOCAL_MODE" != "0" && "$MODEL_ID" == */* && "$MODEL_ID" != /* && "$MODEL_ID" != ./* ]]; then
  SNAPSHOT_ROOT="$CACHE_DIR/models--${MODEL_ID//\//--}/snapshots"
  if [[ -d "$SNAPSHOT_ROOT" ]]; then
    LOCAL_SNAPSHOT="$(find "$SNAPSHOT_ROOT" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
  fi
fi

if [[ -n "$LOCAL_SNAPSHOT" ]]; then
  if [[ ! -d "$LOCAL_SNAPSHOT" ]]; then
    echo "QWEN_MODEL_LOCAL_SNAPSHOT does not exist: $LOCAL_SNAPSHOT" >&2
    exit 1
  fi
  MODEL_ID="$LOCAL_SNAPSHOT"
  export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
  export TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
elif [[ "$LOCAL_MODE" == "1" ]]; then
  echo "No local Qwen snapshot found for $MODEL_ID under $CACHE_DIR" >&2
  exit 1
fi

source /home/step/anaconda3/etc/profile.d/conda.sh
conda activate qwen-vla

export PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}"
export HF_HOME="${HF_HOME:-$CACHE_DIR}"
export HUGGINGFACE_HUB_CACHE="${HUGGINGFACE_HUB_CACHE:-$CACHE_DIR}"
export HF_ENDPOINT="$ENDPOINT"

exec python -m parksim.vla.qwen_service   --host "$HOST"   --port "$PORT"   --model-id "$MODEL_ID"   --device-map "$DEVICE_MAP"   --torch-dtype "$DTYPE"   --max-new-tokens "$MAX_NEW_TOKENS"   "$@"
