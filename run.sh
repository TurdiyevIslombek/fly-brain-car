#!/bin/bash
# Starts the car app.   bash run.sh          (robot on USB)
#                       bash run.sh --fake   (no robot: simulated car)
cd "$(dirname "$0")"
exec .venv/bin/python app.py "$@"
