#!/usr/bin/env bash
# Copyright 2026 Dental-VLA project. Apache-2.0.
# (Re)install the VR-teleop runtime into .venv at the exact versions that were verified.
set -euo pipefail
cd "$(dirname "$0")/../.."
[[ -x .venv/bin/python ]] || uv venv .venv
uv pip install --python .venv/bin/python -e . \
  "dora-rs-cli==0.5.0" "dora-rs==0.5.0" "openarm-mujoco==2.2.0" "openarm-control==0.4.0" \
  "git+https://github.com/enactic/dora-openarm-kinematics@357eb4f0a43a0af5e92aceb7148d04c67163d13c" \
  "git+https://github.com/enactic/dora-openarm-mujoco@70486da61cd80ee70e645b15a574fd0e850ee380" \
  "git+https://github.com/enactic/dora-openarm-quitter.git@bc9fee210417ca5ec86083913d3281370f1c5241" \
  "git+https://github.com/enactic/dora-openarm-vr.git@18268942572c3bb229453d3d0ddb2033bec95e8d" \
  pillow pure-python-adb
ls .venv/bin | grep dora
