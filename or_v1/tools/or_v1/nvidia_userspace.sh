#!/usr/bin/env bash
# Copyright 2026 Dental-VLA project. Apache-2.0.
# Extract the NVIDIA userspace libraries that match the RUNNING kernel module into
# ~/.local/nvidia-<ver>/lib (no root, nothing installed system-wide). Used by start_vr.sh when
# apt upgraded the driver but the machine has not been rebooted yet.
set -euo pipefail
KVER=$(sed -n 's/.*Kernel Module *\([0-9.]*\).*/\1/p' /proc/driver/nvidia/version)
D="$HOME/.local/nvidia-$KVER"; mkdir -p "$D/lib"; cd "$D"
[[ -f NVIDIA-$KVER.run ]] || curl -fL -o "NVIDIA-$KVER.run" \
  "https://download.nvidia.com/XFree86/Linux-x86_64/$KVER/NVIDIA-Linux-x86_64-$KVER.run"
[[ -d ext ]] || sh "NVIDIA-$KVER.run" --extract-only --target ext >/dev/null
cd lib
for f in ../ext/lib*.so.$KVER; do ln -sf "$f" "$(basename "$f")"; done
ln -sf "libGLX_nvidia.so.$KVER" libGLX_nvidia.so.0; ln -sf "libEGL_nvidia.so.$KVER" libEGL_nvidia.so.0
ln -sf "libcuda.so.$KVER" libcuda.so.1; ln -sf libcuda.so.1 libcuda.so; ln -sf "libnvidia-ml.so.$KVER" libnvidia-ml.so.1
echo "ok: $D/lib"
