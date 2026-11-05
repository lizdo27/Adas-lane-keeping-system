"""Vectorized, camera-relative preview geometry. No vehicle pose/odometry claim."""
import numpy as np


def predictions(corridor, angles, half_width):
    """Score ALL steering arcs on the same observed samples, without truncation.

    Returns minimum clearance, limiting side, normalized lateral residual at
    the near preview, and tangent residual (radians), for each angle.
    """
    xs, ys, slope, width, guard, edge = corridor
    angles = np.atleast_1d(angles).astype(float)
    k = np.tan(np.deg2rad(angles))[:, None] / .256
    z = k * xs[None, :]
    valid = np.all(np.abs(z) < .98, axis=1)
    root = np.sqrt(np.maximum(1-z*z, 1e-12))
    yp = k * xs[None, :]**2 / (1+root)
    normal = np.sqrt(1+slope*slope)
    if guard == 'both':
        offset = (yp-ys[None, :]) / normal[None, :]
        gaps = width/2-half_width-np.abs(offset)
        ids = np.argmin(gaps, axis=1)
        sides = np.where(offset[np.arange(len(angles)), ids] > 0, 'left', 'right')
    else:
        signed = edge[None, :]-yp if guard == 'left' else yp-edge[None, :]
        gaps = signed/normal[None, :]-half_width
        sides = np.full(len(angles), guard)
    near = xs <= xs[0]+.12
    position = np.mean((ys[None, near]-yp[:, near])/normal[None, near], axis=1)
    heading = np.mean(np.arctan(slope[None, near])-np.arcsin(np.clip(z[:, near], -.98, .98)), axis=1)
    clearance = np.min(gaps, axis=1)
    clearance[~valid] = np.nan
    position[~valid] = np.nan
    heading[~valid] = np.nan
    return clearance, sides, position, heading


def alignment_cost(position, heading):
    # Metres at the observed near preview, plus a tangent term scaled in metres.
    return position**2 + (.18*heading)**2
