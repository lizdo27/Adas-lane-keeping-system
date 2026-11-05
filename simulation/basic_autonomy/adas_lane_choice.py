"""Resolve a slanted divider near the image axis for right-lane assistance.

A short visible sign alone can put a divider on the wrong side of the car.
Require a long straight observed segment before projecting to the front axle;
never reinterpret a bend, a junction branch, or an unseen line.
"""
from copy import deepcopy
import numpy as np
from vroom_lane import offset_boundary,describe_visibility


def prefer_right_lane(result,config):
    r=result
    if r.get('branch_groups') or r.get('path_limit_m') is not None:return r
    if r.get('status')!='TWO_BOUNDARIES':return r
    right=next((b for b in r.get('boundaries',[]) if b.get('side')=='right'),None)
    if right is None or right['lo']>.53:return r
    p=np.asarray(right.get('path',[]),float)
    if p.ndim!=2 or len(p)<12:return r
    p=p[p[:,0]<=p[0,0]+.35]
    if len(p)<12 or np.ptp(p[:,0])<.25:return r
    slope,intercept=np.polyfit(p[:,0]-.256,p[:,1],1)
    residual=np.max(np.abs(p[:,1]-(slope*(p[:,0]-.256)+intercept)))
    if abs(slope)>.85 or residual>.004 or intercept<.10:return r
    # The selected 'right' divider is clearly LEFT of the front axle under the
    # bounded straight-segment model. Do not center on the adjacent left lane.
    b=deepcopy(right);b['side']='left'
    center=offset_boundary(b,config.lane_width_m,'left').tolist()
    if not center:return r
    out=deepcopy(r)
    out.update(boundaries=[b],center=center,center_source='configured_width',
               measured_lane_width_m=None,status='ONE_BOUNDARY_ESTIMATE',
               reason='Right-lane preference: observed straight divider is left of front axle',
               lane_selection='right_of_observed_divider',pair_resolution=None)
    return describe_visibility(out)
