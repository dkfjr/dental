#!/usr/bin/env bash
# Copyright 2026 Dental-VLA project. Apache-2.0.
# One-command VR teleop for the OR scene v1.0:
#   dora (udp-receiver -> ik -> gripper -> mujoco viewer -> recorder) in the background,
#   CloudXR web bridge in the foreground. Ctrl+C stops both.
#
#   tools/or_v1/start_vr.sh                 # headset worn on the head (default)
#   tools/or_v1/start_vr.sh --neck-drop 0   # headset hung at the neck (enactic style)
#   tools/or_v1/start_vr.sh --record out/session.csv   # also log controller data
#
# Then on the Quest browser: https://<PC-IP>:48322/client/#/sim  -> Start.
# Stand like the viewer's start pose (ㄴ: elbows 90 deg, controllers forward), press left Y
# to calibrate; the robot holds still until then. Y again = re-calibrate.
set -euo pipefail
cd "$(dirname "$0")/../.."
export DISPLAY=${DISPLAY:-:1}
mkdir -p out
if ss -uln | grep -q ':5006 '; then
  echo "[start_vr] port 5006 busy — another dora/teleop is running. Stop it first (Ctrl+C in its terminal)."; exit 1
fi
# NVIDIA driver/library mismatch without reboot (2026-09-29: apt installed 595 while the
# kernel still runs 590.48 -> no OpenGL for new processes). If the kernel module version has a
# matching userspace copy extracted in ~/.local/nvidia-<ver>/lib (no root; see
# tools/or_v1/nvidia_userspace.sh), use it. After a reboot the versions match and this is a no-op.
KVER=$(sed -n 's/.*Kernel Module *\([0-9.]*\).*/\1/p' /proc/driver/nvidia/version 2>/dev/null || true)
if [[ -n "$KVER" && ! -e /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.$KVER ]]; then
  if [[ -d "$HOME/.local/nvidia-$KVER/lib" ]]; then
    export LD_LIBRARY_PATH="$HOME/.local/nvidia-$KVER/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    echo "[start_vr] NVIDIA kernel $KVER != system libs -> using ~/.local/nvidia-$KVER/lib (reboot fixes this)"
  else
    echo "[start_vr] NVIDIA kernel $KVER != system libs and no local copy -> run tools/or_v1/nvidia_userspace.sh"; exit 1
  fi
fi
# CloudXR runtime + web client (port 48322). It was started by hand on 2026-09-01 and does
# not survive a reboot, so start it here when it is not running.
if ! ss -tln | grep -q ':48322 '; then
  echo "[start_vr] CloudXR not running -> starting it (log: out/cloudxr.log)"
  LOG="$PWD/out/cloudxr.log"
  (cd "$HOME/vr_teleop" && PATH="$HOME/vr_teleop/cloudxr_service_venv/bin:/usr/local/cuda-12.8/bin:$PATH" \
      LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}/usr/local/cuda-12.8/lib64" \
      setsid nohup python -m isaacteleop.cloudxr --accept-eula --host-client > "$LOG" 2>&1 < /dev/null &)
  for _ in $(seq 60); do ss -tln | grep -q ':48322 ' && break; sleep 1; done
  ss -tln | grep -q ':48322 ' || { echo "[start_vr] CloudXR did not come up, see out/cloudxr.log"; exit 1; }
fi
# Use the project venv directly. `uv run` may silently RE-CREATE .venv (it did on 2026-09-29
# after a uv update), wiping dora + the enactic nodes, so it is not used here.
export VIRTUAL_ENV="$PWD/.venv" PATH="$PWD/.venv/bin:$PATH"
if [[ ! -x .venv/bin/dora || ! -x .venv/bin/dora-openarm-mujoco ]]; then
  echo "[start_vr] dora/enactic nodes missing in .venv -> run: tools/or_v1/install_teleop.sh"; exit 1
fi
dora run config/dataflow-mujoco-or-v1.yaml > out/dora.log 2>&1 &
DORA=$!
cleanup() { kill -INT "$DORA" 2>/dev/null || true; wait "$DORA" 2>/dev/null || true; echo "[start_vr] stopped"; }
trap cleanup EXIT
for _ in $(seq 60); do ss -uln | grep -q ':5006 ' && break; sleep 0.5; done
echo "[start_vr] dora running (log: out/dora.log, video: out/videos/). IP: $(hostname -I | awk '{print $1}')"
echo "[start_vr] Quest browser -> https://$(hostname -I | awk '{print $1}'):48322/client/#/sim"
mkdir -p out/sessions
REC=out/sessions/vr_$(date +%Y%m%d_%H%M%S).csv
echo "[start_vr] controller log: $REC"
echo "[start_vr] MuJoCo 초기 자세처럼 ㄴ자(팔꿈치 90°, 컨트롤러 앞으로)로 서서 컨트롤러 옆 그립 버튼 -> 로봇이 따라오기 시작 (재보정: 왼쪽 Y)"
tools/or_v1/vr_web_bridge.sh --print --record "$REC" "$@"
