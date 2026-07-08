#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONPATH="$PWD/python${PYTHONPATH:+:$PYTHONPATH}"
python3 -m parksim.vla.smoke
python3 -m parksim.vla.http_smoke
python3 -m parksim.vla.service_smoke
