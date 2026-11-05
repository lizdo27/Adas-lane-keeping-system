"""Associate one CURRENT observed edge independently of lane-center visibility.

Association never supplies an old curve. Unknown side follows an established
edge only through uninterrupted, uniquely matching CURRENT observations.
Missing/jumping observations or a nearby branch break the chain. The caller
resets it on camera time gaps. This is image continuity, not odometry.
"""
from copy import deepcopy
import math
import numpy as np
from vroom_curves import curve_y, curve_slope


class EdgeGuard:
    def __init__(self):
        self.reset()

    def reset(self):
        self.edge = None
        self.identity_edge = None
        self.count = 0
        self.source = ''
        self.pair_refs=[];self.pair_count=0

    @staticmethod
    def usable(boundary, limit):
        try:
            lo = max(.45, float(boundary['lo']))
            hi = min(.8, float(boundary['hi']), limit)
            if not math.isfinite(lo + hi) or hi - lo < .10:
                return False
            rmse = float(boundary.get('rmse', 0.))
            if not math.isfinite(rmse) or rmse > .025:
                return False
            xs = np.linspace(lo, hi, 16)
            return bool(np.isfinite(curve_y(boundary, xs)).all()
                        and np.isfinite(curve_slope(boundary, xs)).all())
        except (KeyError, ValueError, TypeError, IndexError):
            return False

    @staticmethod
    def matches(a, b):
        lo = max(.45, a['lo'], b['lo'])
        hi = min(.75, a['hi'], b['hi'])
        if hi - lo < .04:
            return False
        xs = np.linspace(lo, hi, 16)
        error = np.abs(curve_y(a, xs) - curve_y(b, xs))
        angles = np.abs(np.arctan(curve_slope(a, xs)) - np.arctan(curve_slope(b, xs)))
        if np.max(error)<.065 and np.max(angles)<.20:return True
        # Dashed endpoints can change the local derivative while the observed
        # curves remain within centimetres. Confirm identity with the shared
        # segment's direction, not an isolated endpoint tangent. No gap filling.
        if hi-lo>=.16 and np.max(error)<.015:
            slope_a=np.polyfit(xs-xs.mean(),curve_y(a,xs),1)[0]
            slope_b=np.polyfit(xs-xs.mean(),curve_y(b,xs),1)[0]
            return bool(abs(math.atan(slope_a)-math.atan(slope_b))<.12)
        return False

    def update(self, result, now):
        previous_pair=self.pair_refs;previous_count=self.pair_count
        pair=result.get('boundaries',[])
        pair_valid=(result.get('status')=='TWO_BOUNDARIES' and result.get('tracking_state')=='CONSISTENT'
                    and not result.get('branch_groups') and result.get('path_limit_m') is None
                    and len(pair)==2 and {b.get('side') for b in pair}=={'left','right'})
        self.pair_refs=[];self.pair_count=0
        if pair_valid:
            continuous=all(sum(old.get('side')==b.get('side') and self.matches(old,b) for old in previous_pair)==1 for b in pair)
            self.pair_count=previous_count+1 if continuous else 1
            self.pair_refs=deepcopy(pair)
            # Store a confirmed PAIR without arbitrarily choosing one of its edges.
            if self.edge is None:return
        elif len(pair)==1 and previous_count>=3 and result.get('status') in ('ONE_BOUNDARY_ESTIMATE','UNCERTAIN_SIDE','UNCERTAIN_GEOMETRY'):
            candidates=[b for b in previous_pair if self.matches(b,pair[0])
                        and pair[0].get('side') in (b.get('side'),'unknown')]
            if len(candidates)==1:
                self.edge=deepcopy(candidates[0]);self.identity_edge=deepcopy(candidates[0]);self.count=previous_count
        limit = result.get('path_limit_m')
        if result.get('branch_groups') and limit is None:
            self.reset(); return
        # Detector already ends path_limit 0.08 m BEFORE the observed contact.
        # Do not subtract another 0.10 m and discard a visible approach.
        limit = float(limit) if limit is not None else .8
        bounds = result.get('boundaries', [])
        if (result.get('status')=='AMBIGUOUS_BRANCH' and result.get('path_limit_m') is not None
                and self.edge is None and len(bounds)>1):
            # Seed only a UNIQUE currently visible approach before the measured
            # junction limit. Never choose an exit beyond that limit.
            approaches=[b for b in bounds if b.get('side') in ('left','right')
                        and b['lo']<=.53 and self.usable(b,limit)]
            if len(approaches)==1:bounds=approaches
        if (result.get('status')=='AMBIGUOUS_BRANCH' and result.get('path_limit_m') is not None
                and self.edge is not None and self.count>=1 and len(bounds)>1):
            # Follow only the uniquely matching CURRENT approach edge. Never
            # select an exit or keep an old curve when the edge disappears.
            # Tentative matches may accumulate; corridor() still requires
            # THREE consecutive observations before supplying any assistance.
            bounds=[b for b in bounds if b.get('side')==self.edge['side']
                    and self.usable(b,.8) and self.matches(self.identity_edge,b)]
        # Preserve an established edge across a confirmed pair and back to one
        # visible edge. Require a unique CURRENT match, never reuse its old path.
        if result.get('status')=='TWO_BOUNDARIES' and self.edge is not None:
            bounds=[b for b in bounds if b.get('side')==self.edge['side']
                    and self.usable(b,.8) and self.matches(self.identity_edge,b)]
        if result.get('status')=='UNCERTAIN_GEOMETRY' and (self.edge is None or len(bounds)!=1 or not self.matches(self.edge,bounds[0])):
            self.reset();return
        # Do not choose between branches.
        allowed = ('ONE_BOUNDARY_ESTIMATE', 'ONE_BOUNDARY_TRACKED',
                   'UNCERTAIN_SIDE', 'AMBIGUOUS_BRANCH', 'ONE_BOUNDARY_APPROACH', 'TWO_BOUNDARIES', 'UNCERTAIN_GEOMETRY')
        if (result.get('status') not in allowed or len(bounds) != 1
                or not self.usable(bounds[0], .8)):
            self.reset(); return
        b = deepcopy(bounds[0])
        identity = deepcopy(b)
        if result.get('path_limit_m') is not None:
            path=np.asarray(b.get('path',[]),dtype=float)
            if path.ndim!=2 or path.shape[1]!=2:
                self.reset();return
            path=path[path[:,0]<=limit]
            # A short current approach can preserve identity for the NEXT image,
            # but corridor/front projection still require 0.10 m support.
            if len(path)<4 or np.ptp(path[:,0])<.04 or b.get('rmse',1.)>.015:
                self.reset();return
            b.update(path=path.tolist(),lo=float(path[0,0]),hi=float(path[-1,0]))
        known = b.get('side') in ('left', 'right')
        matched = self.edge is not None and self.matches(self.identity_edge, identity)
        if matched and known and b['side'] != self.edge['side']:
            # A sudden side flip is not evidence that it is safe to steer across.
            self.reset(); return
        if not known:
            if not matched or self.count < 3:
                self.reset(); return
            b['side'] = self.edge['side']
            self.source = 'continuous_observed_edge'
        else:
            self.source = 'observed_edge_identity'
        self.count = self.count + 1 if matched else 1
        self.edge = b
        self.identity_edge = deepcopy(b)

    def corridor(self, result, half_width):
        if result.get('status')=='TWO_BOUNDARIES' or self.edge is None or self.count < 3:
            return None
        b = self.edge
        lo = max(.45, b['lo']); hi = min(.8, b['hi'])
        limit = result.get('path_limit_m')
        if limit is not None:
            hi = min(hi, limit)
        if hi - lo < .10:
            return None
        xs = np.linspace(lo, hi, 24)
        edge = curve_y(b, xs); slope = curve_slope(b, xs)
        # Virtual inward reference is only for mark overlap. It is NOT an
        # inferred opposite edge, nor a route to follow autonomously.
        sign = 1 if b['side'] == 'left' else -1
        ys = edge - sign * (half_width + .035) * np.sqrt(1 + slope*slope)
        for mark in result.get('transverse_markings', []):
            if lo <= mark['x'] <= hi:
                cy = float(np.interp(mark['x'], xs, ys))
                if mark['right'] <= cy <= mark['left']:
                    return None
        return xs, ys, slope, None, b['side'], edge
