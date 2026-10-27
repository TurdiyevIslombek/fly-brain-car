"""Looming from camera pictures: when something comes closer, its image EXPANDS.
Optical flow -> fit u = a + b*x + c*y, v = d + e*x + f*y on the moving pixels -> expansion = b + f.
Left half of the picture -> left eye, right half -> right eye.

Expansion is measured PER SECOND (not per frame), so the eyes give the same answer at 10 or 30 fps.
An object coming at you with "time to contact" T seconds expands at about 2/T per second."""
import numpy as np
import cv2
import config

EXP_RATE_SAT = getattr(config, "EXP_RATE_SAT", 2.0)      # expansion per second that gives MAX_HZ
IGNORE_BOTTOM = getattr(config, "CAM_IGNORE_BOTTOM", 0.3)  # skip the nearest floor (bottom 30%)


def flow(prev_gray, gray):
    return cv2.calcOpticalFlowFarneback(prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)


def expansion(f, min_motion=0.3):
    h, w = f.shape[:2]
    ys, xs = np.mgrid[0:h, 0:w]
    moving = np.hypot(f[..., 0], f[..., 1]) > min_motion
    if moving.sum() < 30:
        return 0.0
    X = np.stack([np.ones(moving.sum()), xs[moving], ys[moving]], axis=1)
    coef_u = np.linalg.lstsq(X, f[..., 0][moving], rcond=None)[0]
    coef_v = np.linalg.lstsq(X, f[..., 1][moving], rcond=None)[0]
    return float(coef_u[1] + coef_v[2])


def looming_from_frames(prev_gray, gray, dt=0.05):
    """Two small gray pictures taken dt seconds apart ->
    (left eye Hz, right eye Hz, {"exp_L", "exp_R": expansion per second, "motion": image motion in px/s})."""
    dt = min(max(dt, 0.02), 0.3)
    f = flow(prev_gray, gray)
    # is the car moving? the floor in the bottom part of the picture slides by whenever it drives
    # (90th percentile: most of a blank wall gives no flow at all, so the median was always ~0)
    floor = f[int(f.shape[0] * 0.6):]
    motion = float(np.percentile(np.hypot(floor[..., 0], floor[..., 1]), 90)) / dt
    top = f[: int(f.shape[0] * (1 - IGNORE_BOTTOM))]                     # the floor right in front expands
    half = top.shape[1] // 2                                             # whenever the car drives: skip it
    exp_L, exp_R = expansion(top[:, :half]) / dt, expansion(top[:, half:]) / dt
    to_hz = lambda e: float(config.MAX_HZ * np.clip(e / EXP_RATE_SAT, 0, 1))
    return to_hz(exp_L), to_hz(exp_R), {"exp_L": exp_L, "exp_R": exp_R, "motion": motion}
