#!/usr/bin/env bash
# Copyright 2026 Dental-VLA project. Apache-2.0.
# Regenerate the MuJoCo OR scene from the Blender file.
#   tools/or_v1/export_all.sh            # after editing scenes/dental_or_v1.blend
#   tools/or_v1/export_all.sh --rebuild  # also re-run robot swap + tray builder (overwrites tray edits)
set -euo pipefail
cd "$(dirname "$0")/../.."
BLEND=scenes/dental_or_v1.blend
PY=.venv/bin/python
if [[ "${1:-}" == "--rebuild" ]]; then
  $PY tools/or_v1/build_scene.py robot-json
  blender -b "$BLEND" -P tools/or_v1/blender_swap_robot.py
  blender -b "$BLEND" -P tools/or_v1/blender_build_tray.py
fi
blender -b "$BLEND" -P tools/or_v1/blender_export_env.py
$PY tools/or_v1/build_scene.py scene
