# Copyright 2026 Dental-VLA project. Apache-2.0.
"""Hypothesis test: do controllers arrive once the OpenXR app actually renders frames?
Opens ONE OpenXR session via Televiz (kXr), shows a coloured panel 1.2 m ahead, shares the
session with DeviceIO (head + controllers) and prints validity for ~15 s.
Run with tools/or_v1/vr_probe.sh-style env:  tools/or_v1/vr_focus_test.sh"""
import time
import torch
import isaacteleop.deviceio as dio
import isaacteleop.viz as viz
from isaacteleop.oxr import OpenXRSessionHandles

head, ctl = dio.HeadTracker(), dio.ControllerTracker()
tr = [head, ctl]
cfg = viz.VizSessionConfig()
cfg.mode = viz.DisplayMode.kXr
cfg.app_name = "DentalVLA-FocusTest"
cfg.required_extensions = list(dio.DeviceIOSession.get_required_extensions(tr))
cfg.clear_color = (0.05, 0.08, 0.12, 1.0)
vs = viz.VizSession.create(cfg)
lc = viz.QuadLayerConfig()
lc.name = "panel"; lc.format = viz.PixelFormat.kRGBA8; lc.resolution = viz.Resolution(256, 144)
lc.placement = viz.QuadLayerPlacement(viz.Pose3D((0.0, 1.4, -1.2), (1, 0, 0, 0)), (1.6, 0.9))
layer = vs.add_quad_layer(lc)
img = torch.zeros((144, 256, 4), dtype=torch.uint8, device="cuda"); img[..., 2] = 200; img[..., 3] = 255
with dio.DeviceIOSession.run(tr, OpenXRSessionHandles(*vs.get_oxr_handles())) as s:
    t0 = last = time.time(); frames = 0
    while time.time() - t0 < 15:
        layer.submit(img)
        vs.render(); frames += 1
        s.update()
        if time.time() - last > 1.0:
            last = time.time()
            h = head.get_head(s); h = getattr(h, "data", h)
            out = [f"state={vs.get_state()} fps={frames:3d} head={'OK' if h is not None and h.is_valid else '--'}"]
            frames = 0
            for side, fn in (("L", ctl.get_left_controller), ("R", ctl.get_right_controller)):
                c = fn(s); c = getattr(c, "data", c)
                if c is None:
                    out.append(f"{side}=None"); continue
                out.append(f"{side}: grip={'OK' if c.grip_pose.is_valid else '--'} trig={c.inputs.trigger_value:.2f} Y/B={c.inputs.secondary_click}")
            print("  ".join(out), flush=True)
vs.destroy()
