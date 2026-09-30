#!/usr/bin/env bash
# Launch the ESP-SDR S3 web GUI. Creates a local venv on first run.
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -d venv ]]; then
    python3 -m venv venv
    ./venv/bin/pip install -q --upgrade pip
    ./venv/bin/pip install -q -r requirements.txt
fi
exec ./venv/bin/python server.py
