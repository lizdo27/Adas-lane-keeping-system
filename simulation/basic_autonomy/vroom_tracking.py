"""Short-lived association of CURRENT visible paths; never dead reckoning.

No vehicle command or ROS dependency. This checks continuity without odometry,
so large motion reacquires rather than projecting an old path into a new frame.
"""
from collections import deque
from copy import deepcopy
import math
import numpy as np
from vroom_lane import offset_boundary,describe_visibility
from vroom_identity import recent_side


def paths_agree(a,b):
    a=np.asarray(a);b=np.asarray(b)
    if len(a)<8 or len(b)<8:return False
    a=a[np.argsort(a[:,0])];b=b[np.argsort(b[:,0])]
    lo=max(a[0,0],b[0,0]);hi=min(a[-1,0],b[-1,0])
    if hi-lo<.10:return False
    xs=np.linspace(lo,hi,20)
    error=np.abs(np.interp(xs,a[:,0],a[:,1])-np.interp(xs,b[:,0],b[:,1]))
    return bool(np.median(error)<.07 and np.max(error)<.12)


class LaneTracker:
    def __init__(self,max_wall_gap_s=.5):
        if not math.isfinite(max_wall_gap_s) or not .5<=max_wall_gap_s<=1.2:
            raise ValueError('Tracking wall gap must be in [.5,1.2] seconds')
        self.max_wall_gap_s=max_wall_gap_s
        self.reset()

    def reset(self):
        self.stamp=None;self.last_time=None;self.last=[];self.good=0
        self.pending=[];self.pending_count=0;self.widths=deque(maxlen=9)
        self.width_time=None
        self.side_references=[]

    def update(self,result,stamp,now):
        return describe_visibility(self._update(result,stamp,now))

    def _update(self,result,stamp,now):
        """stamp is source time in seconds, now is monotonic wall receipt time.

        Duplicate source frames never warm up or refresh stored width. A gap or
        clock reversal discards association. No observation means no output path.
        """
        if not math.isfinite(stamp) or not math.isfinite(now):raise ValueError('Invalid tracking time')
        out=deepcopy(result)
        out['observation_status']=result['status']
        if self.stamp is not None and stamp==self.stamp:
            out.update(center=[],center_source='none',tracking_state='DUPLICATE_FRAME')
            return out
        if self.stamp is not None and (stamp<self.stamp or stamp-self.stamp>.5 or now-self.last_time>self.max_wall_gap_s or now<self.last_time):
            self.reset()
        self.stamp=stamp
        if self.width_time is not None and now-self.width_time>1.:
            self.widths.clear();self.width_time=None
        if out['status'] not in ('TWO_BOUNDARIES','ONE_BOUNDARY_ESTIMATE','UNCERTAIN_SIDE'):
            self.side_references=[]
        # Recover only the side of ONE CURRENT visible edge. A stale path is
        # never returned, and this estimate cannot refresh its own identity.
        if out['status']=='UNCERTAIN_SIDE' and len(out['boundaries'])==1:
            b=out['boundaries'][0]
            match=recent_side(b,self.side_references,stamp,now)
            if match is not None:
                side,source_age,wall_age=match
                width=out['config']['lane_width_m']
                path=offset_boundary(b,width,side).tolist()
                if path:
                    out.update(status='ONE_BOUNDARY_TRACKED',center=path,center_source='configured_width',
                               boundaries=[dict(b,side=side)],
                               side_association=dict(source_age_s=source_age,wall_age_s=wall_age),
                               reason=f'Recent {side} edge identity ({source_age:.2f}s); width {width:.2f}m estimate')
        if not out['center']:
            if out['status']!='UNCERTAIN_SIDE':self.side_references=[]
            self.pending=[];self.pending_count=0;self.good=0
            out['tracking_state']='NO_OBSERVATION'
            # Keep a reference for at most 0.5 s, never return its old center.
            if self.last_time is None:self.last_time=now
            return out
        if out['status'] in ('ONE_BOUNDARY_ESTIMATE','ONE_BOUNDARY_TRACKED') and len(self.widths)>=5 and np.std(self.widths)<.02:
            b=out['boundaries'][0];width=float(np.median(self.widths))
            out['center']=offset_boundary(b,width,b['side']).tolist()
            out.update(center_source='recent_measured_width',used_lane_width_m=width,
                       reason=f'One visible edge; recent observed lane width {width:.2f}m (estimate)')
            if not out['center']:
                self.good=0;self.pending=[];self.pending_count=0
                self.side_references=[]
                out.update(status='UNCERTAIN_GEOMETRY',center_source='none',tracking_state='NO_OBSERVATION',
                           reason='Recent-width offset folds; no forward center')
                return out
        current=out['center']
        if self.last and not paths_agree(self.last,current):
            self.side_references=[]
            self.pending_count=self.pending_count+1 if paths_agree(self.pending,current) else 1
            self.pending=deepcopy(current)
            self.widths.clear();self.width_time=None
            if self.pending_count<3:
                out.update(center=[],center_source='none',tracking_state='REACQUIRING',
                           status='TRACKING_UNCERTAIN',reason='Observed path jumped; waiting for consistent fresh frames')
                return out
            self.good=0
        self.pending=[];self.pending_count=0
        self.good+=1;self.last=deepcopy(current);self.last_time=now
        out['tracking_state']='CONSISTENT' if self.good>=3 else 'ACQUIRING'
        if self.good>=3 and out['status'] in ('TWO_BOUNDARIES','ONE_BOUNDARY_ESTIMATE'):
            self.side_references=[dict(boundary=deepcopy(b),stamp=stamp,now=now)
                                  for b in out['boundaries'] if b.get('side') in ('left','right')]
        elif out['status'] not in ('ONE_BOUNDARY_TRACKED','TWO_BOUNDARIES','ONE_BOUNDARY_ESTIMATE'):
            self.side_references=[]
        # Learn only from a stable pair away from an identified branch.
        width=out.get('measured_lane_width_m')
        if out['status']=='TWO_BOUNDARIES' and self.good>=3 and width is not None:
            if math.isfinite(width) and .20<=width<=.65:
                if self.widths and abs(width-float(np.median(self.widths)))>.04:self.widths.clear()
                self.widths.append(width);self.width_time=now
        return out
