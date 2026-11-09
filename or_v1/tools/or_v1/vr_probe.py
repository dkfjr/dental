# Copyright 2026 Dental-VLA project. Apache-2.0.
"""Raw IsaacTeleop probe: is the head / each controller (grip & aim pose) valid? inputs?
    tools/or_v1/vr_web_bridge.sh is not needed; run with the CloudXR venv via vr_probe.sh"""
import time
import isaacteleop.deviceio as dio
import isaacteleop.oxr as oxr

head, ctl, hand = dio.HeadTracker(), dio.ControllerTracker(), dio.HandTracker()
tr = [head, ctl, hand]
with oxr.OpenXRSession("DentalVLA-Probe", dio.DeviceIOSession.get_required_extensions(tr)) as o, \
        dio.DeviceIOSession.run(tr, o.get_handles()) as s:
    t0 = time.time()
    while time.time() - t0 < 6:
        s.update()
        h = head.get_head(s); h = getattr(h, "data", h)
        out = [f"head={('%+.3f,%+.3f,%+.3f' % (h.pose.position.x, h.pose.position.y, h.pose.position.z)) if h is not None and h.is_valid else '--'}"]
        for side, fn in (("L", ctl.get_left_controller), ("R", ctl.get_right_controller)):
            c = fn(s); raw = c; c = getattr(c, "data", c)
            if c is None:
                out.append(f"{side}: None(tracked={getattr(raw, 'is_tracked', getattr(raw, 'is_active', '?'))})"); continue
            g, a, i = c.grip_pose, c.aim_pose, c.inputs
            out.append(f"{side}: grip={'OK' if g and g.is_valid else '--'} aim={'OK' if a and a.is_valid else '--'} "
                       f"trig={getattr(i, 'trigger_value', 0):.2f} Y/B={getattr(i, 'secondary_click', 0)}")
        for side, fn in (("handL", hand.get_left_hand), ("handR", hand.get_right_hand)):
            hd = fn(s); hd = getattr(hd, "data", hd)
            ok = hd is not None and bool(getattr(hd, "is_active", getattr(hd, "is_valid", False)))
            out.append(f"{side}={'OK' if ok else '--'}")
        print("  ".join(out), flush=True)
        time.sleep(0.5)
