"""Bounded association of a current visible edge with a recently confirmed side.

No odometry, prediction or extrapolated center. History only supplies a side label.
"""
import math
import numpy as np
from vroom_curves import curve_y,curve_slope


def same_visible_edge(a,b):
    if abs(a['lo']-b['lo'])>.08:return False
    lo=max(a['lo'],b['lo']);hi=min(a['hi'],b['hi'])
    if hi-lo<.16:return False
    xs=np.linspace(lo,hi,24)
    sa=curve_slope(a,xs);sb=curve_slope(b,xs)
    distance=np.abs(curve_y(a,xs)-curve_y(b,xs))/np.sqrt(1+sa*sa)
    angles=np.abs(np.arctan(sa)-np.arctan(sb))
    return bool(np.median(distance)<.025 and np.max(distance)<.04
                and np.max(angles)<math.radians(10))


def recent_side(boundary,references,stamp,now):
    """All times are seconds. Uncertain results must never refresh references."""
    matches=[]
    for ref in references:
        source_age=stamp-ref['stamp'];wall_age=now-ref['now']
        if not (0<source_age<=.5 and 0<=wall_age<=1.5):continue
        known=ref['boundary']
        if known.get('side') not in ('left','right'):continue
        if same_visible_edge(known,boundary):
            matches.append((known['side'],source_age,wall_age))
    # Even two matching references with the same label are ambiguous identities.
    return matches[0] if len(matches)==1 else None
