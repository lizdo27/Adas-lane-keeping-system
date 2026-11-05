"""Camera lane-departure assistance. No ROS, route planner or autonomous throttle.

Clearance is a forward spatial estimate using a constant-steering bicycle path,
not measured tyre overlap, time-to-collision or a guarantee against departure.
"""
from copy import deepcopy
import math
import numpy as np
from adas_edges import EdgeGuard
from adas_near_guard import limit_correction
from adas_guidance import predictions,alignment_cost
from adas_recovery import front_estimate,recovery_plan,pulse_scores
from vroom_curves import curve_y,curve_slope


class LaneAssist:
    def __init__(self, enabled=True, timeout=.8, half_width=.10, margin=.035,
                 max_correction=50., rate=90.):
        values=(timeout,half_width,margin,max_correction,rate)
        if not all(math.isfinite(x) for x in values):raise ValueError('Non-finite ADAS setting')
        if not (.2<=timeout<=1.2 and .07<=half_width<=.15 and .01<=margin<=.08
                and 0<max_correction<=50 and 0<rate<=90):raise ValueError('Invalid ADAS setting')
        self.enabled=enabled;self.timeout=timeout;self.half_width=half_width
        self.margin=margin;self.max_correction=max_correction;self.rate=rate
        self.edges=EdgeGuard()
        self.result=None;self.received=None;self.stamp=None;self.good=0
        self._corridor_cache=None;self.front=dict(state='UNKNOWN',lines={});self.check_time=0.;self.front_recovering=False
        self.correction=0.;self.last_tick=None;self.active=False;self.departure_active=False;self.align_active=False;self._plan_cache=None
        self.front_conflict=False
        self.near_edge_caution=False
        self.info=dict(state='UNAVAILABLE',reason='Waiting for lane')
        self.observation_issue=None

    def reset(self,reason='Driver stop'):
        self.front_recovering=False
        self.correction=0.;self.last_tick=None;self.active=False;self.departure_active=False;self.align_active=False;self._plan_cache=None
        self.front_conflict=False
        self.info=dict(state='MANUAL',reason=reason)

    def invalidate(self,reason):
        self.observation_issue=reason
        self.result=None;self.received=None;self.stamp=None;self.good=0
        self._corridor_cache=None;self.front=dict(state='UNKNOWN',lines={})
        self.edges.reset();self.near_edge_caution=False;self.reset(reason)

    def toggle(self):
        self.enabled=not self.enabled;self.reset('Assist ON' if self.enabled else 'Assist OFF')

    def observe(self,result,stamp,received,now):
        self.check_time=now;self._corridor_cache=None;self.front=dict(state='UNKNOWN',lines={})
        self.front_conflict=False
        if (not all(math.isfinite(x) for x in (stamp,received,now))
                or not 0<=now-received<=self.timeout
                or (self.stamp is not None and stamp<=self.stamp)):
            self.observation_issue='Invalid/stale observation'
            self.result=None;self.good=0;self.edges.reset();self.near_edge_caution=False;self.reset(self.observation_issue);return
        self.observation_issue=None
        if self.received is not None and (not 0<received-self.received<=self.timeout or stamp-self.stamp>.5):
            self.good=0;self.edges.reset();self.near_edge_caution=False
        self._plan_cache=None
        self.stamp=stamp;self.received=received;self.result=deepcopy(result)
        self.edges.update(result,received)
        self.good=self.good+1 if result.get('tracking_state')=='CONSISTENT' else 0
        self.front=front_estimate(self.result,self.edges.edge if self.edges.count>=3 else None,self.good,self.half_width)
        if self.near_edge_caution and self.good>=3 and result.get('status')=='TWO_BOUNDARIES':
            clear=[]
            for b in result.get('boundaries',[]):
                lo=max(.45,b['lo']);hi=min(.65,b['hi'],lo+.12)
                if b.get('side') not in ('left','right') or lo>.53 or hi-lo<.1:continue
                xs=np.linspace(lo,hi,16);edge=curve_y(b,xs);slope=curve_slope(b,xs)
                gap=predictions((xs,edge,slope,None,b['side'],edge),[0],self.half_width)[0][0]
                clear.append(bool(np.isfinite(gap) and gap>.04))
            if len(clear)==2 and all(clear):self.near_edge_caution=False

    @staticmethod
    def pair_continues_across_mark(result, x):
        """Permit a transverse stripe only with two directly observed edges.

        A marking alone is not a junction. Both low-error edge fits must span
        either side of it, remain separate, and have matching local tangents.
        """
        bounds=result.get('boundaries',[])
        sides={b.get('side'):b for b in bounds}
        if len(bounds)!=2 or set(sides)!={'left','right'}:return False
        xs=np.linspace(x-.04,x+.04,9)
        for b in bounds:
            if b['lo']>xs[0] or b['hi']<xs[-1] or not math.isfinite(b.get('rmse',float('nan'))) or b['rmse']>.015:return False
            path=np.asarray(b.get('path',[]))
            if path.ndim!=2 or len(path)<8 or np.max(np.diff(path[:,0]))>.12:return False
        left,right=sides['left'],sides['right']
        sl=curve_slope(left,xs);sr=curve_slope(right,xs)
        width=(curve_y(left,xs)-curve_y(right,xs))/np.sqrt(1+((sl+sr)/2)**2)
        return bool(np.isfinite(width).all() and np.min(width)>.22 and np.max(width)<.55
                    and np.ptp(width)<.035 and np.max(np.abs(np.arctan(sl)-np.arctan(sr)))<.2)

    def corridor(self,now):
        r=self.result
        if r is None or self.received is None:return None,self.observation_issue or 'No lane result'
        if not 0<=now-self.received<=self.timeout:return None,'Processed image stale'
        # Geometry is immutable between observations. Freshness is checked
        # above on EVERY call, including cached unavailable results.
        if self._corridor_cache is None:self._corridor_cache=self._build_corridor()
        return self._corridor_cache

    def _build_corridor(self):
        r=self.result
        edge_corridor=self.edges.corridor(r,self.half_width)
        if edge_corridor is not None:return edge_corridor,''
        if r.get('branch_groups') or r.get('path_limit_m') is not None:return None,'Junction / no confirmed approach edge: driver control'
        if r.get('status') not in ('TWO_BOUNDARIES','ONE_BOUNDARY_ESTIMATE','ONE_BOUNDARY_TRACKED'):
            return None,'Lane identity uncertain'
        guard='both'
        if r.get('center_source')=='two_boundaries':width=r.get('measured_lane_width_m')
        else:
            return None,'Visible edge identity warming up / uncertain'
        if guard=='both' and (not isinstance(width,(int,float)) or not math.isfinite(width) or not .22<=width<=.55):
            return None,'Lane width uncertain'
        if self.good<3 or r.get('tracking_state')!='CONSISTENT':return None,'Lane warming up'
        if r.get('center_visibility')!='VISIBLE':return None,'Near lane not visible'
        path=np.asarray(r.get('visible_center',[]),dtype=float)
        if (path.ndim!=2 or path.shape[1]!=2 or len(path)<3 or not np.isfinite(path).all()
                or np.any(np.diff(path[:,0])<=0)):
            return None,'Invalid lane path'
        lo=max(.45,float(path[0,0]));hi=min(.8,float(path[-1,0]))
        if hi-lo<.10:return None,'Not enough near support'
        xs=np.linspace(lo,hi,24);ys=np.interp(xs,path[:,0],path[:,1])
        slope=np.gradient(ys,xs)
        for mark in r.get('transverse_markings',[]):
            if lo<=mark['x']<=hi:
                cy=float(np.interp(mark['x'],xs,ys))
                if mark['right']<=cy<=mark['left'] and not self.pair_continues_across_mark(r,mark['x']):
                    return None,'Transverse marking without confirmed continuous edges: driver control'
        return (xs,ys,slope,width,guard,None),''

    def clearance(self,corridor,steer):
        gaps,sides,_,_=predictions(corridor,[steer],self.half_width)
        return float(gaps[0]),str(sides[0])

    def command(self,speed,steer,now,connected=True):
        velocity,output=self._command(speed,steer,now,connected)
        if velocity>0 and self.enabled and self.info.get('state')=='ASSIST':
            guarded,limited=limit_correction(self.result or {},steer,output,self.half_width)
            if limited:
                output=guarded;self.correction=output-steer;self.near_edge_caution=True
                self.info.update(reason='Near visible edge limits distant-path correction',
                                 near_edge_limited=True,residual_risk=True)
        return velocity,output

    def _command(self,speed,steer,now,connected=True):
        """Return driver speed unchanged, except invalid/disconnected input stops.

        The caller enforces driver key expiry and Space. ADAS never invents a
        positive speed. Reverse, disabled assist and uncertain images pass the
        driver's steering through, without retaining an old correction.
        """
        self.check_time=now
        if not all(math.isfinite(x) for x in (speed,steer,now)) or abs(speed)>.08 or abs(steer)>25 or not connected:
            self.reset('Invalid input / no subscriber');return 0.,0.
        if speed==0:
            self.reset('Driver stopped');return 0.,0.
        if speed<0 or not self.enabled:
            self.reset('Reverse: driver control' if speed<0 else 'Assist OFF');return speed,steer
        dt=.05 if self.last_tick is None else now-self.last_tick
        self.last_tick=now
        if not 0<dt<=.25:
            self.reset('Control loop gap');self.info['state']='UNAVAILABLE';return speed,steer
        corridor,reason=self.corridor(now)
        contact=self.front.get('clearance_m',float('inf'))
        fresh=(self.result is not None and self.received is not None
               and 0<=now-self.received<=self.timeout)
        has_edge=bool(self.front.get('lines'))
        imminent=False
        # Begin earlier on the visible approach before a branch removes support.
        approach=bool(self.result and self.result.get('path_limit_m') is not None)
        if fresh and has_edge and contact<(.05 if approach else .035):
            future,_,_=pulse_scores(self.front,[steer],self.half_width)
            imminent=bool(future[0]<.015)
        self.front_recovering=bool(fresh and has_edge and
                                  (contact<(.015 if self.front_recovering else .005) or imminent))
        self.front_conflict=False
        if self.front_recovering and not self.front.get('paired') and corridor is not None:
            gaps,sides,_,_=predictions(corridor,[steer],self.half_width)
            # A backwards projection of ONE edge must not overrule a conflicting
            # observed corridor. On bends the unseen continuation can be wrong.
            if float(gaps[0])<self.margin and str(sides[0])!=self.front.get('side'):
                self.front_conflict=True;self.front_recovering=False
        if self.front_recovering:
            output=self._recover_front(speed,steer,dt)
            if output is not None:return output
        if corridor is None:
            self.front_recovering=False
            self.correction=0.;self.active=False;self.departure_active=False;self.align_active=False;self._plan_cache=None
            self.info=dict(state='UNAVAILABLE',reason=reason);return speed,steer
        guard=corridor[4]
        xs,ys,slope,width,_,edge=corridor
        if guard!='both' and self.near_edge_caution and not has_edge and np.max(np.abs(slope))>.7:
            # A far curved edge alone cannot establish clearance on the hidden
            # inside of a bend. Do not pull toward that unobserved side.
            self.correction=0.;self.active=False;self._plan_cache=None
            self.info=dict(state='UNAVAILABLE',reason='Recently threatened near edge lost: driver must correct')
            return speed,steer
        margin=self.margin if guard!='both' else min(self.margin,max(.005,(width/2-self.half_width)*.6))
        recovery=False
        if guard=='both':
            # Near departure recovery is a short action, reevaluated on each
            # fresh image, not one fixed steering angle over the entire bend.
            near=xs<=min(.70,float(xs[0])+.16)
            if np.count_nonzero(near)>=3 and np.ptp(xs[near])>=.10:
                local=(xs[near],ys[near],slope[near],width,guard,None)
                if self.clearance(local,steer)[0]<margin:
                    corridor=local;recovery=True
        correction_limit=self.max_correction
        low=max(-25.,steer-correction_limit);high=min(25.,steer+correction_limit)
        # Compare on a shared near interval supported by the REQUESTED turn.
        # Farther extreme candidates must not shrink every score to 0.5 m.
        xs,ys,slope,width,guard,edge=corridor
        driver_k=abs(math.tan(math.radians(steer)))/.256
        common=driver_k*xs<.98
        if np.count_nonzero(common)<3 or np.ptp(xs[common])<.06:
            self.correction=0.;self.active=False;self.departure_active=False;self.align_active=False;self._plan_cache=None
            self.info=dict(state='UNAVAILABLE',reason='Turn leaves observable preview')
            return speed,steer
        corridor=(xs[common],ys[common],slope[common],width,guard,
                  edge[common] if edge is not None else None)
        # Every considered candidate must cover this ENTIRE interval.
        feasible=math.degrees(math.atan(.979*.256/float(xs[common][-1])))
        low=max(low,-feasible);high=min(high,feasible)
        gaps,sides,positions,headings=predictions(corridor,[steer],self.half_width)
        gap=float(gaps[0]);side=str(sides[0]);pos=float(positions[0]);head=float(headings[0])
        # Explicit A/D remains the driver's choice unless it risks a boundary.
        # W can assist small unintended drift before the departure threshold.
        align=False
        if guard=='both' and abs(steer)<=5:
            pos_limit=.012 if self.align_active else .025
            head_limit=math.radians(2 if self.align_active else 4)
            align=abs(pos)>pos_limit or abs(head)>head_limit
        self.align_active=align
        threshold=margin+(min(.02,margin*.5) if self.departure_active else 0.)
        risk=gap<threshold
        self.departure_active=risk
        if not risk and not align:
            self.correction=0.;self.active=False
            self.info=dict(state='MONITOR',reason='Driver path within lane' if guard=='both' else 'Watching visible '+guard+' edge only',
                           clearance_m=gap,guard=guard,preview_offset_m=pos,preview_heading_deg=math.degrees(head))
            return speed,steer
        xs,ys,_,_,_,_=corridor;x=float(xs[-1]);y=float(ys[-1])
        center_angle=math.degrees(math.atan2(.512*y,x*x+y*y))
        key=(self.stamp,steer,guard,recovery,align,risk,float(xs[0]),float(xs[-1]))
        if self._plan_cache is not None and self._plan_cache[0]==key:
            desired,best_gap,desired_cost=self._plan_cache[1:]
        else:
            candidates=np.unique(np.r_[np.linspace(low,high,81),steer,np.clip(center_angle,low,high)])
            gaps,_,positions,headings=predictions(corridor,candidates,self.half_width)
            valid=np.isfinite(gaps);candidates=candidates[valid];gaps=gaps[valid]
            costs=alignment_cost(positions[valid],headings[valid])
            target_margin=margin+(.01 if guard!='both' else min(.005,margin*.2))
            if guard=='both':
                # Fit position AND lane tangent; discard paths that approach an
                # edge too closely merely to reduce the heading error.
                floor=target_margin if risk else min(margin,gap)
                safe=gaps>=floor
                if not np.any(safe):safe=gaps>=float(np.max(gaps))-.002
                ids=np.flatnonzero(safe)
                j=int(ids[np.argmin(costs[ids])])
            else:
                safe=np.flatnonzero(gaps>=target_margin)
                j=int(safe[np.argmin(np.abs(candidates[safe]-steer))]) if len(safe) else int(np.argmax(gaps))
            desired=float(candidates[j]);best_gap=float(gaps[j]);desired_cost=float(costs[j])
            self._plan_cache=(key,desired,best_gap,desired_cost)
        improves_alignment=align and desired_cost<alignment_cost(pos,head)-1e-6 and best_gap>=min(margin,gap)-1e-6
        if best_gap<=gap+.003 and not improves_alignment:
            self.correction=0.;self.active=False
            self.info=dict(state='WARNING',reason='No reliable correction',side=side,clearance_m=gap,residual_risk=risk)
            return speed,steer
        target=desired-steer;step=self.rate*dt
        correction=self.correction+float(np.clip(target-self.correction,-step,step))
        correction=float(np.clip(correction,-correction_limit,correction_limit))
        output=float(np.clip(steer+correction,-25.,25.))
        # Driver reversal can make the previous signed correction inappropriate.
        out_gaps,_,out_pos,out_head=predictions(corridor,[output],self.half_width)
        out_gap=float(out_gaps[0])
        alignment_worse=improves_alignment and (out_gap<min(margin,gap)-1e-6 or alignment_cost(out_pos[0],out_head[0])>alignment_cost(pos,head)+1e-9)
        if not math.isfinite(out_gap) or alignment_worse or (out_gap<gap and not improves_alignment):
            correction=0.;output=steer;out_gap=gap
        self.correction=correction;self.active=True
        self.info=dict(state='ASSIST' if correction else 'WARNING',reason=('Align position and heading' if align and not risk else 'Departure risk '+side)+(' (one visible edge)' if guard!='both' else '')+(' / correction limited' if best_gap<margin else ''),
                       side=side,guard=guard,preview_x_m=[float(xs[0]),float(xs[-1])],clearance_m=gap,predicted_corrected_clearance_m=out_gap,
                       correction_deg=correction,limited=best_gap<margin, residual_risk=out_gap<margin,
                       identity_source=self.edges.source if guard!='both' else 'paired_boundaries',
                       desired_steer_deg=desired,recovery=recovery,margin_m=margin,
                       guidance_mode="CENTER_AND_HEADING" if guard=="both" else "VISIBLE_EDGE_ONLY",
                       preview_offset_m=pos,preview_heading_deg=math.degrees(head),
                       driver_steer_deg=steer,output_steer_deg=output)
        return speed,output

    def _recover_front(self,speed,steer,dt):
        key=('front',self.stamp,steer)
        if self._plan_cache is not None and self._plan_cache[0]==key:plan=self._plan_cache[1]
        else:
            plan=recovery_plan(self.front,steer,self.half_width,self.max_correction)
            self._plan_cache=(key,plan)
        if plan is None:return None
        target=plan['desired']-steer
        correction=self.correction+float(np.clip(target-self.correction,-self.rate*dt,self.rate*dt))
        correction=float(np.clip(correction,-self.max_correction,self.max_correction))
        output=float(np.clip(steer+correction,-25,25))
        gaps,costs,valid=pulse_scores(self.front,[output],self.half_width,reference_angle=steer)
        if not valid[0] or gaps[0]<plan['driver_gap']-1e-6:
            correction=0.;output=steer
            gaps,_,_=pulse_scores(self.front,[output],self.half_width,reference_angle=steer)
        self.correction=correction;self.active=bool(correction)
        self.info=dict(state='ASSIST' if correction else 'WARNING',reason='Recover estimated front-edge contact',
                       guard='both' if self.front.get('paired') else self.front['side'],
                       side=self.front['side'],guidance_mode='FRONT_EDGE_RECOVERY',
                       desired_steer_deg=plan['desired'],driver_steer_deg=steer,output_steer_deg=output,
                       correction_deg=correction,clearance_m=self.front['clearance_m'],
                       predicted_corrected_clearance_m=float(gaps[0]),residual_risk=bool(gaps[0]<.015),
                       recovery=True,limited=plan['gap']<.015,recovery_travel_m=.15,margin_m=.015,
                       interpretation=('Front envelope estimate; rear tyres not assessed' if self.front.get('paired')
                                       else 'Observed front edge only; opposite edge and rear tyres not assessed'))
        return speed,output

    def snapshot(self,now=None):
        now=self.check_time if now is None else now
        alert={k:deepcopy(v) for k,v in self.front.items() if k!='lines'}
        if self.result is None or self.received is None or not math.isfinite(now) or not 0<=now-self.received<=self.timeout:
            alert=dict(state='UNKNOWN',reason='No fresh near-vehicle estimate')
        if self.front_conflict:alert=dict(state='UNKNOWN',reason='Front projection conflicts with observed corridor')
        return dict(deepcopy(self.info),enabled=self.enabled,observation_stamp_s=self.stamp,
                    correction_deg=self.correction,autonomous_throttle=False,lane_alert=alert)
