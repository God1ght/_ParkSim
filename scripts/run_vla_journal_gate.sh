#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <suite_dir> [paper_gate args...]" >&2
  exit 2
fi
SUITE_DIR="$1"
shift

exec env PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.paper_gate "$SUITE_DIR" --profile trc --strict "$@"
