"""Short front-envelope recovery from a bounded camera extrapolation.

This estimates the front axle envelope, NOT measured tyre contact or the rear
of the car. No map pose, odometry, throttle or hidden-edge reconstruction.
"""
import math
import numpy as np

WHEELBASE=.256


def front_estimate(result,edge,good,half_width):
    unknown=dict(state='UNKNOWN',reason='Near vehicle edge not observable',lines={})
    paired=(result.get('status')=='TWO_BOUNDARIES' and result.get('center_source')=='two_boundaries'
            and result.get('tracking_state')=='CONSISTENT' and good>=3
            and not result.get('branch_groups') and result.get('path_limit_m') is None)
    bounds=result.get('boundaries',[]) if paired else ([edge] if edge is not None else [])
    lines={}
    for boundary in bounds:
        side=boundary.get('side')
        if side not in ('left','right'):continue
        rmse=boundary.get('rmse',float('inf'))
        if not math.isfinite(rmse) or rmse>.015:continue
        p=np.asarray(boundary.get('path',[]),dtype=float)
        if p.ndim!=2 or p.shape[1]!=2 or len(p)<8 or not np.isfinite(p).all():continue
        if np.any(np.diff(p[:,0])<=0):continue
        p=p[p[:,0]<=p[0,0]+.16]
        if len(p)<8 or np.ptp(p[:,0])<.10 or not 0<=p[0,0]-WHEELBASE<=.30:continue
        slope,intercept=np.polyfit(p[:,0]-WHEELBASE,p[:,1],1)
        residual=np.abs(p[:,1]-(slope*(p[:,0]-WHEELBASE)+intercept))
        # Segment directions resist dashed-endpoint derivative spikes. The
        # residual bound still rejects curved or poorly supported projections.
        mid=len(p)//2
        tangents=np.arctan([np.polyfit(q[:,0],q[:,1],1)[0]
                            for q in (p[:mid+1],p[mid:])])
        if np.max(residual)>.0025 or np.ptp(tangents)>.12 or abs(slope)>.7:continue
        lines[side]=dict(slope=float(slope),intercept=float(intercept),lo=float(p[0,0]),hi=float(p[-1,0]))
    if not lines:return unknown
    if len(lines)==2:
        l,r=lines['left'],lines['right']
        width=(l['intercept']-r['intercept'])/math.hypot(1,(l['slope']+r['slope'])/2)
        if not .22<=width<=.55 or abs(math.atan(l['slope'])-math.atan(r['slope']))>.1:return unknown
    gaps={side:((line['intercept'] if side=='left' else -line['intercept'])-half_width)/math.hypot(1,line['slope'])
          for side,line in lines.items()}
    side=min(gaps,key=gaps.get);gap=gaps[side]
    return dict(state='ESTIMATED_OVERLAP' if gap<=0 else 'NEAR_EDGE' if gap<.015 else 'NO_FRONT_OVERLAP_ESTIMATED',
                side=side,clearance_m=gap,lines=lines,paired=len(lines)==2,
                reason='Bounded straight-edge projection to front axle; rear tyres not assessed')


def pulse_scores(model,angles,half_width,reference_angle=None):
    """Predict a 0.15 m rear-axle travel pulse, including rotated front corners.

Initial overlap cannot be undone at s=0. Require no further penetration and
score the clearance after the first 0.10 m; never choose a route at a junction.
"""
    angles=np.atleast_1d(angles).astype(float)
    count=len(angles)
    if reference_angle is not None:angles=np.r_[angles,reference_angle]
    s=np.linspace(0,.15,16)
    k=np.tan(np.deg2rad(angles))[:,None]/WHEELBASE;theta=k*s
    rear_x=s*np.sinc(theta/np.pi)
    rear_y=k*s*s/2*np.sinc(theta/(2*np.pi))**2
    x=rear_x+WHEELBASE*np.cos(theta);y=rear_y+WHEELBASE*np.sin(theta)
    valid=np.ones(len(angles),bool);gaps=[]
    for side,line in model['lines'].items():
        sign=1 if side=='left' else -1
        corner_x=x-sign*half_width*np.sin(theta);corner_y=y+sign*half_width*np.cos(theta)
        edge=line['intercept']+line['slope']*(corner_x-WHEELBASE)
        gap=sign*(edge-corner_y)/math.hypot(1,line['slope']);gaps.append(gap)
        valid&=np.all((corner_x>=line['lo']-.30)&(corner_x<=line['hi']),axis=1)
        initial=gap[:,0]
        # The already-contacting side may start negative, the opposite side
        # must keep its clearance; avoid a larger initial cut toward either.
        if reference_angle is None:
            valid&=np.min(gap,axis=1)>=np.minimum(initial,.005)-.001
        else:
            # During actuator slew, improve on the driver's whole trajectory;
            # otherwise an initially insufficient small turn is reset forever.
            valid&=np.all(gap>=np.minimum(gap[-1],.005)-.001,axis=1)
    gaps=np.stack(gaps);future=np.min(gaps[:,:,s>=.10],axis=(0,2))
    if model.get('paired'):
        left,right=model['lines']['left'],model['lines']['right']
        slope=(left['slope']+right['slope'])/2
        center=(left['intercept']+right['intercept'])/2+slope*(x[:,-1]-WHEELBASE)
        pos=(center-y[:,-1])/math.hypot(1,slope)
    else:
        # Only clear the observed edge. This is not a reconstructed lane center.
        line=next(iter(model['lines'].values()));slope=line['slope']
        pos=future-.025
    heading=math.atan(slope)-theta[:,-1]
    cost=pos**2+(.18*heading)**2
    return future[:count],cost[:count],valid[:count]


def recovery_plan(model,driver,half_width,limit):
    if not model.get('lines'):return None
    low=max(-25.,driver-limit);high=min(25.,driver+limit)
    candidates=np.unique(np.r_[np.linspace(low,high,101),driver])
    gap,cost,valid=pulse_scores(model,candidates,half_width)
    base_gap,base_cost,base_valid=pulse_scores(model,[driver],half_width)
    # Respect an intentional turn already clearing the contact with both
    # front corners checked. W still receives position/heading correction.
    if abs(driver)>5 and base_valid[0] and base_gap[0]>=.015:
        return dict(desired=driver,driver_gap=float(base_gap[0]),gap=float(base_gap[0]),driver_cost=float(base_cost[0]),cost=float(base_cost[0]))
    good=valid&(gap>=max(model['clearance_m'],float(base_gap[0]))+.002)
    if not np.any(good):return None
    safe=good&(gap>=.015)
    if not np.any(safe):safe=good&(gap>=np.max(gap[good])-.002)
    ids=np.flatnonzero(safe);j=ids[np.argmin(cost[ids])]
    return dict(desired=float(candidates[j]),driver_gap=float(base_gap[0]),gap=float(gap[j]),driver_cost=float(base_cost[0]),cost=float(cost[j]))
