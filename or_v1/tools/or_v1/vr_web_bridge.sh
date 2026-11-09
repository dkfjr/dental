#!/usr/bin/env bash
# Copyright 2026 Dental-VLA project. Apache-2.0.
# Wireless VR (web): Quest browser -> https://<PC-IP>:48322/client/#/sim (CloudXR / IsaacTeleop)
#   -> this bridge -> UDP 127.0.0.1:5006 -> dora (config/dataflow-mujoco-or-v1.yaml)
# Extra args go to the bridge, e.g.  tools/or_v1/vr_web_bridge.sh --print --sx 1
set -euo pipefail
cd "$(dirname "$0")/../.."
export CXR_INSTALL_DIR=$HOME/.cloudxr
export XR_RUNTIME_JSON=$HOME/.cloudxr/openxr_cloudxr.json
export NV_CXR_RUNTIME_DIR=$HOME/.cloudxr/run
export NV_CXR_OUTPUT_DIR=$HOME/.cloudxr/logs
export NV_CXR_ENABLE_PUSH_DEVICES=true
export NV_CXR_ENABLE_TENSOR_DATA=true
# default: follow controller orientation like the original enactic app; later args override
exec "${CXR_PY:-$HOME/vr_teleop/cloudxr_service_venv/bin/python3}" -u src/dental_vla/isaacteleop_bridge.py --wrist-mode controller "$@"
