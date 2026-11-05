"""Veto assistance that worsens a separately observed near boundary.

Only current, labelled observations are used. This does not infer tyre contact
or fill the blind area between the camera's ground view and the vehicle.
"""
import numpy as np
from vroom_curves import curve_y,curve_slope
from adas_guidance import predictions


def limit_correction(result,driver,output,half_width):
    if result.get('status')!='TWO_BOUNDARIES' or result.get('tracking_state')!='CONSISTENT':
        return output,False
    bounds=result.get('boundaries',[])
    if len(bounds)!=2 or {b.get('side') for b in bounds}!={'left','right'}:return output,False
    constraints=[]
    for b in bounds:
        lo=max(.45,b['lo']);hi=min(.65,b['hi'],lo+.12)
        if lo>.53 or hi-lo<.06 or b.get('rmse',1)>.015:continue
        xs=np.linspace(lo,hi,16);edge=curve_y(b,xs);slope=curve_slope(b,xs)
        corridor=(xs,edge,slope,None,b['side'],edge)
        base=predictions(corridor,[driver],half_width)[0][0]
        proposed=predictions(corridor,[output],half_width)[0][0]
        if np.isfinite(base) and base<.035 and (not np.isfinite(proposed) or proposed<base-.001):
            constraints.append((corridor,base))
    if not constraints:return output,False
    # Move back toward the driver's command, never create a stronger turn or
    # jump to the opposite lock from a veto. Preserve all near-edge constraints.
    angles=np.linspace(output,driver,101);valid=np.ones(len(angles),dtype=bool)
    for corridor,base in constraints:
        gap=predictions(corridor,angles,half_width)[0]
        valid&=np.isfinite(gap)&(gap>=base-.001)
    ids=np.flatnonzero(valid)
    return (float(angles[ids[0]]) if len(ids) else driver),True
