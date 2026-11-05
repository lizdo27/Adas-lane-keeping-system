"""VROOM-inspired scanline/ground/DBSCAN lane diagnostics. No ROS or commands.

Reimplementation of Machines 13 (2025) 514, sec. 4.4.1, not authors' code.
Threshold defaults are our starting values, not values reported by the paper.
Coordinates: x forward from rear axle, y left, metres; flat ground assumption.
"""
from dataclasses import dataclass, asdict, fields
import json
import math
import time
import cv2
import numpy as np
from lane_control import Camera
from vroom_curves import cut_corners,smooth_track,curve_y,curve_slope,curve_points,match_curves,short_branch_contact


@dataclass
class Config:
    height_m: float = .208449
    pitch_deg: float = 22.
    hfov_deg: float = 62.2
    forward_m: float = .25193
    left_m: float = .0025
    scan_rows: int = 64
    roi_top: float = .20
    roi_bottom: float = .95
    edge_step: float = 12.
    min_contrast: float = 35.
    min_brightness: float = 100.
    plateau_tolerance: float = .25
    plateau_fraction: float = .55
    max_rise_pixels: int = 5  # at 640px width
    stripe_min_m: float = .006
    stripe_max_m: float = .085
    near_m: float = .30
    far_m: float = 1.5
    dbscan_eps_m: float = .10
    dbscan_min_samples: int = 3
    min_cluster_points: int = 6
    min_span_m: float = .16
    max_fit_error_m: float = .035
    lane_width_m: float = .35  # measured/configured fallback, not a detection
    lane_min_m: float = .20
    lane_max_m: float = .65
    reference_m: float = .65

    def validate(self):
        for f in fields(self):
            if not math.isfinite(getattr(self,f.name)):
                raise ValueError(f'Non-finite config: {f.name}')
        for name in ('scan_rows','max_rise_pixels','dbscan_min_samples','min_cluster_points'):
            if type(getattr(self,name)) is not int: raise ValueError(f'{name} must be an integer')
        if not (8<=self.scan_rows<=160 and 0<=self.roi_top<self.roi_bottom<=1):
            raise ValueError('Invalid scan rows or ROI')
        if not (.01<self.height_m<1 and 0<self.pitch_deg<80 and 10<self.hfov_deg<150):
            raise ValueError('Invalid camera geometry')
        if not (0<self.near_m<self.reference_m<self.far_m<=5): raise ValueError('Invalid ground range')
        if not (0<self.stripe_min_m<self.stripe_max_m<.2): raise ValueError('Invalid stripe width')
        if not (0<self.lane_min_m<self.lane_width_m<self.lane_max_m<2): raise ValueError('Invalid lane width')
        if not (0<self.plateau_fraction<=1 and 0<self.plateau_tolerance<1): raise ValueError('Invalid plateau settings')
        if not (0<self.edge_step<255 and 0<self.min_contrast<255 and 0<=self.min_brightness<=255):
            raise ValueError('Invalid intensity settings')
        if not (1<=self.max_rise_pixels<=20 and 0<self.dbscan_eps_m<self.lane_min_m):
            raise ValueError('Invalid rise/cluster radius')
        if not (2<=self.dbscan_min_samples<=20 and 3<=self.min_cluster_points<=100): raise ValueError('Invalid cluster size')
        if not (0<self.min_span_m<self.far_m-self.near_m and 0<self.max_fit_error_m<.2):
            raise ValueError('Invalid fit settings')
        return self

    @classmethod
    def load(cls,path=None):
        values=json.loads(path.read_text()) if path else {}
        return cls(**values).validate()

    def camera(self):
        return Camera(self.height_m,math.radians(self.pitch_deg),math.radians(self.hfov_deg),self.forward_m,self.left_m)


def cluster_points(points,eps,min_samples):
    """Small deterministic DBSCAN; capped input, no new sklearn dependency."""
    n=len(points)
    labels=np.full(n,-1,dtype=int)
    if not n:return labels
    distance=np.sum((points[:,None,:]-points[None,:,:])**2,axis=2)
    neighbors=[np.flatnonzero(row<=eps*eps) for row in distance]
    visited=np.zeros(n,dtype=bool);group=0
    for i in range(n):
        if visited[i]:continue
        visited[i]=True
        if len(neighbors[i])<min_samples:continue
        labels[i]=group;queue=list(neighbors[i]);queued=set(queue)
        while queue:
            j=queue.pop()
            if not visited[j]:
                visited[j]=True
                if len(neighbors[j])>=min_samples:
                    for k in neighbors[j]:
                        if k not in queued:queue.append(k);queued.add(k)
            if labels[j]<0:labels[j]=group
        group+=1
    return labels


def plateau_value(sample,tolerance):
    # Approximate Boyer-Moore voting with a relative intensity interval.
    # Uniform plateaus never change the first winner; avoid a Python loop over
    # wide crosswalk stripes without changing the voting result.
    stable=np.abs(sample-float(sample[0]))<=max(4.,tolerance*float(sample[0]))
    if np.all(stable):return float(np.median(sample)),1.
    winner=float(sample[0]);counter=0
    for pixel in sample:
        if counter==0:winner=float(pixel)
        counter+=1 if abs(float(pixel)-winner)<=max(4.,tolerance*winner) else -1
    stable=np.abs(sample-winner)<=max(4.,tolerance*winner)
    return float(np.median(sample[stable])),float(stable.mean())


def split_branches(points):
    """Associate distinct scanline samples before fitting, never average a fork.

    Work near-to-far; continuation uses a short local slope, with one sample per
    row per track. Ambiguous parent clusters are retained as diagnostics only.
    """
    tracks=[]
    for x in np.unique(points[:,0]):
        row=points[np.abs(points[:,0]-x)<=1e-9]
        choices=[]
        for ti,track in enumerate(tracks):
            dx=x-track[-1][0]
            if not 0<dx<=.18:continue
            slope=0.
            if len(track)>=3:
                recent=track[max(0,len(track)-5)]
                slope=float((track[-1][1]-recent[1])/(track[-1][0]-recent[0]))
            for ri,p in enumerate(row):
                error=abs(p[1]-(track[-1][1]+slope*dx))
                if error<=.035+.3*dx and np.linalg.norm(p-track[-1])<=.24:
                    choices.append((error,ti,ri))
        used_tracks=set();used_rows=set()
        for error,ti,ri in sorted(choices):
            if ti in used_tracks or ri in used_rows:continue
            tracks[ti].append(row[ri]);used_tracks.add(ti);used_rows.add(ri)
        for ri,p in enumerate(row):
            if ri not in used_rows:tracks.append([p])
    return [np.asarray(t) for t in tracks]


def supported_span(points,c):
    # Measure observed diagonal extent, while keeping y(x) well supported.
    # Transverse stripes and tiny fragments are still rejected.
    return (len(points)>=c.min_cluster_points and np.ptp(points[:,0])>=.06
            and np.linalg.norm(np.ptp(points,axis=0))>=c.min_span_m)


def fit_boundary(selected,c,group,track):
    if not supported_span(selected,c):return None
    local=smooth_track(selected)
    residual=np.abs(selected[:,1]-local[:,1])
    threshold=max(.008,2.5*float(np.median(residual)))
    inliers=selected[residual<=threshold]
    if len(inliers)<max(c.min_cluster_points,.7*len(selected)) or not supported_span(inliers,c):return None
    local=smooth_track(inliers)
    rmse=float(np.sqrt(np.mean((inliers[:,1]-local[:,1])**2)))
    if rmse>c.max_fit_error_m:return None
    coeff=np.polyfit(inliers[:,0],inliers[:,1],2)
    lo,hi=float(inliers[:,0].min()),float(inliers[:,0].max())
    return dict(group=int(group),track=int(track),coeff=coeff.tolist(),path=local.tolist(),
                fit_kind='local_curve',lo=lo,hi=hi,rmse=rmse,
                y=float(np.interp(c.reference_m,local[:,0],local[:,1])),count=len(inliers))


def recover_dense_edges(xy,eligible,labels,c):
    """Continue established edges through busy rows using actual stripe points.

    A seed must have long, clean support outside the dense band. Extend one row
    at a time with a tight local prediction gate; never choose the outermost
    points blindly, and stop where two continuations are equally plausible.
    """
    recovered=eligible.copy()
    for group in sorted(set(labels)-{-1}):
        for seed in split_branches(xy[labels==group]):
            if fit_boundary(seed,c,group,0) is None:continue
            for direction in (-1,1):
                track=list(seed if direction==1 else seed[::-1])
                rows=sorted(np.unique(xy[~eligible,0]),reverse=direction<0)
                for x in rows:
                    dx=x-track[-1][0]
                    if direction*dx<=0:continue
                    if abs(dx)>.18:break
                    recent=np.asarray(track[-8:])
                    # A local tangent is more stable than extrapolating a
                    # quadratic across a crosswalk with no supporting samples.
                    slope=(recent[-1,1]-recent[0,1])/(recent[-1,0]-recent[0,0])
                    indices=np.flatnonzero((xy[:,0]==x)&~eligible)
                    errors=np.abs(xy[indices,1]-(track[-1][1]+slope*dx))/math.sqrt(1+slope*slope)
                    order=np.argsort(errors)
                    if not len(order) or errors[order[0]]>.012+.04*abs(dx):continue
                    if len(order)>1 and errors[order[1]]-errors[order[0]]<.006:break
                    i=indices[order[0]]
                    if np.linalg.norm(xy[i]-track[-1])>.20:break
                    recovered[i]=True;track.append(xy[i])
    return recovered


def near_side(boundary):
    """Use the closest observed support, not the sign at a distant lookahead.

    The latter switches sides as a curved/slanted edge crosses the image axis.
    A boundary very close to that axis has ambiguous side; don't invent a center.
    This remains a geometric heuristic, not lane identity or localization.
    """
    x=boundary['lo'];y=float(curve_y(boundary,x))
    slope=float(curve_slope(boundary,x))
    distance=y/math.sqrt(1+slope*slope)
    return ('left' if distance>0 else 'right') if abs(distance)>=.035 else None


def resolve_axis_pair(curves,c,branch_groups):
    """Resolve a near-axis edge using ONE unambiguous observed parallel pair.

    One member must have a known side and the other an uncertain near-axis
    side. A slightly farther companion can be used only on their shared visible
    support. Never extrapolate an edge to the axle or choose among lane pairs.
    """
    candidates=[]
    pool=[b for b in curves if b['lo']<=c.reference_m+.20
          and b['group'] not in branch_groups]
    sides=[near_side(b) for b in pool]
    for i,a in enumerate(pool):
        for j in range(i+1,len(pool)):
            b=pool[j];sa,sb=sides[i],sides[j]
            if a['group']==b['group'] or (sa is None)==(sb is None):continue
            unknown=a if sa is None else b
            if unknown['lo']>c.reference_m:continue
            lo=max(a['lo'],b['lo']);hi=min(a['hi'],b['hi'])
            if hi-lo<c.min_span_m:continue
            xs=np.linspace(lo,hi,24);delta=curve_y(a,xs)-curve_y(b,xs)
            if np.all(delta>.05):left,right=a,b;ls,rs=sa,sb
            elif np.all(delta<-.05):left,right=b,a;ls,rs=sb,sa
            else:continue
            if ls not in (None,'left') or rs not in (None,'right'):continue
            center,width=paired_center(left,right,c)
            if not center:continue
            candidates.append(([dict(left,side='left'),dict(right,side='right')],center,width))
    return (candidates[0] if len(candidates)==1 else None),len(candidates)


def offset_boundary(boundary,width,side):
    """Offset by half lane width along the LOCAL NORMAL, not lateral y only."""
    xs=np.linspace(boundary['lo'],boundary['hi'],60)
    ys=curve_y(boundary,xs)
    slope=curve_slope(boundary,xs)
    direction=-1 if side=='left' else 1
    scale=direction*width/(2*np.sqrt(1+slope*slope))
    center=np.column_stack([xs-slope*scale,ys+scale])
    # Too-tight curvature / noisy normals can fold a parallel curve back over
    # itself. Our forward path representation cannot support that geometry.
    dx=np.diff(center[:,0])
    if not np.all(np.isfinite(center)) or np.any(dx<-.005) or np.sum(np.maximum(-dx,0))>.01:
        return np.empty((0,2))
    return center


def paired_center(left,right,c):
    """Find normal intersections inside BOTH observed curve supports.

    Equal image rows/forward coordinates need not overlap far in a sharp bend.
    This matches the two visible edges in metres without extrapolating either.
    """
    return match_curves(left,right,c)


def branch_horizon(tracks,c):
    """Nearest observed junction between substantial tracks, in ground x.

    A shared DBSCAN label alone does not make the entire visible road unusable.
    Only local contacts locate a junction; unknown contacts stay ambiguous.
    """
    substantial=[t for t in tracks if len(t)>=c.min_cluster_points and np.linalg.norm(np.ptp(t,axis=0))>=c.min_span_m]
    contacts=[]
    for i,a in enumerate(substantial):
        for b in substantial[i+1:]:
            overlap=min(a[:,0].max(),b[:,0].max())-max(a[:,0].min(),b[:,0].min())
            if overlap<=.05:continue
            distances=np.linalg.norm(a[:,None,:]-b[None,:,:],axis=2)
            ai,bi=np.unravel_index(np.argmin(distances),distances.shape)
            if distances[ai,bi]<=c.dbscan_eps_m:
                contacts.append(float(min(a[ai,0],b[bi,0])))
    return min(contacts) if contacts else None


def approach_center(selected,track_points,limit,c):
    """Refit two observed edges strictly BEFORE a branch. No chosen exit."""
    if len(selected)!=2 or {b['side'] for b in selected}!={'left','right'}:return [],[],None
    fitted=[]
    for b in selected:
        pts=track_points[(b['group'],b['track'])]
        clean=fit_boundary(pts[pts[:,0]<=limit],c,b['group'],b['track'])
        if clean is None or near_side(clean)!=b['side']:return [],[],None
        fitted.append(dict(clean,side=b['side']))
    sides={b['side']:b for b in fitted}
    center,width=paired_center(sides['left'],sides['right'],c)
    center=[p for p in center if p[0]<=limit]
    if len(center)<8 or np.linalg.norm(np.array(center[-1])-center[0])<c.min_span_m:return [],[],None
    return fitted,center,width


def single_approach(selected,track_points,limit,c):
    """Visible single edge BEFORE a known contact, with an explicitly assumed width.

    Both the supporting boundary and its offset must end before the limit.
    This estimates an approach only; it never extends through or chooses a branch.
    Do not use this as a fallback when a visible pair failed its geometry checks.
    """
    if len(selected)!=1 or not math.isfinite(limit):return [],[]
    b=selected[0]
    if b.get('side') not in ('left','right'):return [],[]
    pts=track_points[(b['group'],b['track'])]
    clean=fit_boundary(pts[pts[:,0]<=limit],c,b['group'],b['track'])
    if clean is None or near_side(clean)!=b['side']:return [],[]
    path=offset_boundary(clean,c.lane_width_m,b['side'])
    # A normal offset can move x forward, beyond even a truncated edge.
    path=path[(path[:,0]>=c.near_m)&(path[:,0]<=limit)]
    if len(path)<8 or np.ptp(path[:,0])<c.min_span_m:return [],[]
    return [dict(clean,side=b['side'])],path.tolist()


def detect(frame,config=None,*,stripe_support=None,stripe_exclusion=None):
    c=(config or Config()).validate();camera=c.camera();started=time.perf_counter()
    if frame is None or frame.ndim!=3 or frame.shape[2]!=3 or min(frame.shape[:2])<64:
        raise ValueError('Expected BGR image at least 64x64')
    h,w=frame.shape[:2];gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # Filter measured stripes, not source pixels: masking the source would
    # introduce artificial brightness transitions at semantic mask boundaries.
    for mask in (stripe_support,stripe_exclusion):
        if mask is not None and mask.shape!=(h,w):
            raise ValueError('Semantic mask must match source image dimensions')
    # Light horizontal smoothing; do not resize/stretch source geometry.
    gray=cv2.GaussianBlur(gray,(3,1),0)
    points=[];pixels=[];row_counts=[];crowded=[];transverse=[]
    rows=np.unique(np.linspace(c.roi_top*(h-1),c.roi_bottom*(h-1),c.scan_rows).astype(int))
    samples=gray[rows].astype(float)
    thresholds=np.maximum(c.min_brightness,np.percentile(samples,20,axis=1)+c.min_contrast)
    for v,row,threshold in zip(rows,samples,thresholds):
        active=row>=threshold
        edges=np.diff(np.r_[False,active,False].astype(int))
        starts=np.flatnonzero(edges==1);ends=np.flatnonzero(edges==-1)-1
        count=0
        for a,b in zip(starts,ends):
            if a<2 or b>=w-2 or b-a<1:continue
            sample=row[a:b+1];plateau,fraction=plateau_value(sample,c.plateau_tolerance)
            margin=max(2,int(round(c.max_rise_pixels*w/640)))
            low_left=float(np.min(row[max(0,a-margin):a]))
            low_right=float(np.min(row[b+1:min(w,b+margin+1)]))
            if fraction<c.plateau_fraction or plateau-max(low_left,low_right)<c.min_contrast:continue
            if np.max(np.diff(row[max(0,a-margin):min(w,b+2)]))<c.edge_step:continue
            reach=np.flatnonzero(sample>=plateau*.8)
            if not len(reach) or reach[0]>margin:continue
            try:
                x,y=camera.ground((a+b)/2,float(v),w,h)
                _,yl=camera.ground(float(a),float(v),w,h)
                _,yr=camera.ground(float(b),float(v),w,h)
            except ValueError:continue
            if not c.near_m<=x<=c.far_m or abs(y)>.9:continue
            if yl-yr>=.18:
                # A broad transverse white marking, not a semantic junction label.
                transverse.append(dict(x=float(x),left=float(yl),right=float(yr)))
            if not c.stripe_min_m<=yl-yr<=c.stripe_max_m:continue
            if stripe_support is not None and np.mean(stripe_support[v,a:b+1]>0)<.5:continue
            if stripe_exclusion is not None and np.mean(stripe_exclusion[v,a:b+1]>0)>.35:continue
            points.append((x,y));pixels.append((int((a+b)/2),int(v)));count+=1
        row_counts.append(count)
        if count>=3:crowded.append(v)
    # Prevent quadratic clustering costs on pathological texture.
    overloaded=len(points)>600
    xy=np.asarray(points[:600],dtype=float).reshape(-1,2)
    # Crosswalk/start-grid rows can bridge both lane edges into one DBSCAN
    # component. Keep their diagnostic points but exclude the dense row band
    # from boundary fitting. This is our added safeguard, not paper parameters.
    labels=np.full(len(xy),-1,dtype=int)
    spacing=(c.roi_bottom-c.roi_top)*h/c.scan_rows
    eligible=np.array([not any(abs(v-dense)<=2*spacing for dense in crowded)
                       for u,v in pixels[:600]],dtype=bool)
    if not overloaded and np.any(eligible):
        labels[eligible]=cluster_points(xy[eligible],c.dbscan_eps_m,c.dbscan_min_samples)
    recovered_count=0
    if crowded and not overloaded and np.any(labels>=0):
        expanded=recover_dense_edges(xy,eligible,labels,c)
        recovered_count=int(np.sum(expanded&~eligible))
        if recovered_count:
            eligible=expanded;labels[:]=-1
            labels[eligible]=cluster_points(xy[eligible],c.dbscan_eps_m,c.dbscan_min_samples)
    curves=[];branch_groups=[];track_points={};horizons={}
    for group in sorted(set(labels)-{-1}):
        points_in_group=xy[labels==group]
        original=split_branches(points_in_group)
        horizon=branch_horizon(original,c)
        short_contact=short_branch_contact(original)
        if short_contact is not None:
            branch_groups.append(int(group))
            horizon=min(horizon,short_contact) if horizon is not None else short_contact
        tracks=[];after_corner=[];corners=[]
        for chain in original:
            pieces,cuts=cut_corners(chain)
            tracks.extend(pieces);after_corner.extend(i>0 for i in range(len(pieces)))
            corners.extend(cuts)
        if corners:
            branch_groups.append(int(group))
            horizon=min(corners+([horizon] if horizon is not None else []))
        horizons[int(group)]=horizon
        # A side road can be long in y but short in x. Measuring only forward
        # span silently missed such branches and offset a center across them.
        substantial=[t for t in tracks if len(t)>=c.min_cluster_points and np.linalg.norm(np.ptp(t,axis=0))>=c.min_span_m]
        # Several substantial overlapping tracks in ONE spatial component are a
        # fork/crossing, not one boundary to average into a polynomial.
        for i,t in enumerate(substantial):
            for other in substantial[i+1:]:
                overlap=min(t[:,0].max(),other[:,0].max())-max(t[:,0].min(),other[:,0].min())
                if overlap>.05:
                    branch_groups.append(int(group));break
        for track,pts in enumerate(tracks):
            track_points[(int(group),track)]=pts
            fitted=fit_boundary(pts,c,group,track)
            if fitted is not None and not after_corner[track]:curves.append(fitted)
    # Require near support for selecting a lane, while allowing distant clean
    # edges to seed dense-band recovery above. No polynomial extrapolation to
    # reference_m is used to assign left/right identity.
    nearby=[b for b in curves if b['lo']<=c.reference_m]
    left=sorted((b for b in nearby if near_side(b)=='left'),key=lambda b:abs(curve_y(b,b['lo'])))
    right=sorted((b for b in nearby if near_side(b)=='right'),key=lambda b:abs(curve_y(b,b['lo'])))
    selected=[];center=[];width=None;path_limit=None;status='NO_LANE';reason='Need valid lane boundaries'
    center_source='none';pair_resolution=None;pair_candidates=0
    pair=None
    if not (left and right) and any(near_side(b) is None for b in nearby):
        pair,pair_candidates=resolve_axis_pair(curves,c,branch_groups)
    if pair is not None:
        selected,center,width=pair;status='TWO_BOUNDARIES';center_source='two_boundaries'
        pair_resolution='relative_observed_pair'
        reason=f'Near-axis identity from unique observed pair; width {width:.2f}m'
    elif pair_candidates>1:
        selected=[dict(b,side=near_side(b) or 'unknown') for b in nearby]
        status='UNCERTAIN_SIDE';reason='Multiple observed lane pairs; no lane identity selected'
    elif left and right:
        l,r=left[0],right[0];selected=[dict(l,side='left'),dict(r,side='right')]
        center,width=paired_center(l,r,c)
        if center:
            status='TWO_BOUNDARIES';reason=f'Normal-matched boundaries; width about {width:.2f}m'
            center_source='two_boundaries'
        else:
            status='PARTIAL_BOUNDARIES'
            reason='Both edges visible but insufficient consistent paired support; no center'
    elif left or right:
        b=(left or right)[0];side='left' if left else 'right';selected=[dict(b,side=side)]
        center=offset_boundary(b,c.lane_width_m,side).tolist();status='ONE_BOUNDARY_ESTIMATE'
        reason=f'Normal-offset estimate using configured width {c.lane_width_m:.2f}m'
        center_source='configured_width'
        if not center:
            status='UNCERTAIN_GEOMETRY';center_source='none'
            reason='Normal offset folds at this curvature; cannot infer a forward center'
    elif nearby:
        selected=[dict(b,side='unknown') for b in nearby]
        status='UNCERTAIN_SIDE';reason='Boundary too close to vehicle axis; side unknown, no center'
    if any(b['group'] in branch_groups for b in selected):
        affected=set(b['group'] for b in selected if b['group'] in branch_groups)
        contacts=[horizons[g] for g in affected]
        center=[];center_source='none';width=None
        status='AMBIGUOUS_BRANCH';reason='Branch has no supported approach; choose direction manually'
        if all(x is not None for x in contacts):
            # End the diagnostic path before the observed contact. This is a
            # geometric margin, not a speed-dependent braking distance.
            path_limit=min(contacts)-.08
            approach,path,measured=approach_center(selected,track_points,path_limit,c)
            if path:
                selected=approach;center=path;width=measured;center_source='two_boundaries'
                status='BRANCH_AHEAD';reason=f'Observed approach only; path ends before branch at x={path_limit:.2f}m'
            elif len(selected)==1:
                approach,path=single_approach(selected,track_points,path_limit,c)
                if path:
                    selected=approach;center=path;center_source='configured_width'
                    status='ONE_BOUNDARY_APPROACH'
                    reason=f'Estimated approach only ({c.lane_width_m:.2f}m width); ends before branch at x={path_limit:.2f}m'
    if overloaded:status='TOO_MANY_POINTS';reason='Texture overload: tune point detection';center=[]
    result=dict(status=status,reason=reason,points=xy.tolist(),pixels=pixels[:600],labels=labels.tolist(),
                boundaries=selected,center=center,center_source=center_source,measured_lane_width_m=width,
                path_limit_m=path_limit,branch_groups=sorted(set(branch_groups)),algorithm_version=12,recovered_points=recovered_count,candidate_count=len(points),cluster_count=len(set(labels)-{-1}),
                dense_markings=bool(crowded),transverse_markings=transverse,
                pair_resolution=pair_resolution,pair_candidates=pair_candidates,
                processing_ms=(time.perf_counter()-started)*1000,config=asdict(c),image_size=[w,h])
    return describe_visibility(result)


def image_point(camera,x,y,w,h):
    f=w/(2*math.tan(camera.hfov/2));c,s=math.cos(camera.pitch),math.sin(camera.pitch)
    dx=x-camera.forward;left=y-camera.left
    forward=c*dx+s*camera.height
    if forward<=0:return None
    u=(w-1)/2-f*left/forward
    v=(h-1)/2+f*(-s*dx+c*camera.height)/forward
    if not (0<=u<w and 0<=v<h):return None
    return int(u),int(v)


def describe_visibility(result):
    """Describe current image support, never label an inferred path as drive-ready.

    Keep the full geometric center for diagnostics; visible_center is only the
    nearest contiguous in-image part, never a bridge between disjoint pieces.
    A longer piece beyond an out-of-view bend cannot replace the near path.
    """
    if 'image_size' not in result:return result
    w,h=result['image_size'];camera=Config(**result['config']).camera()
    runs=[];run=[];visible=0
    for point in result['center']:
        if image_point(camera,*point,w,h) is not None:
            run.append(point);visible+=1
        elif run:runs.append(run);run=[]
    if run:runs.append(run)
    path=min(runs,key=lambda r:r[0][0],default=[])
    length=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
    state=('NO_CENTER' if not result['center'] else 'OUT_OF_VIEW' if not visible
           else 'SHORT_VISIBLE' if length<result['config']['min_span_m'] else 'VISIBLE')
    result.update(center_visibility=state,visible_center=path,visible_center_length_m=length,
                  visible_center_points=visible,visible_center_runs=len(runs),
                  visible_center_selection='nearest_contiguous')
    return result


def annotate(frame,result):
    canvas=frame.copy();h,w=canvas.shape[:2];c=Config(**result['config']);camera=c.camera()
    for pixel,label in zip(result['pixels'],result['labels']):
        color=(0,230,255) if label>=0 else (120,120,120)
        cv2.circle(canvas,tuple(pixel),2,color,-1)
    for boundary in result['boundaries']:
        path=curve_points(boundary)
        pixels=[image_point(camera,x,y,w,h) for x,y in path]
        pixels=[p for p in pixels if p is not None]
        color={'left':(255,180,0),'right':(255,0,255),'unknown':(150,150,150)}[boundary['side']]
        if len(pixels)>1:cv2.polylines(canvas,[np.array(pixels,np.int32)],False,color,2)
    for x,y in result['center']:
        pixel=image_point(camera,x,y,w,h)
        if pixel:cv2.circle(canvas,pixel,3,(0,220,0) if result.get('center_source')=='two_boundaries' else (0,160,255),-1)
    if result['status'] in ('BRANCH_AHEAD','ONE_BOUNDARY_APPROACH') and result['center']:
        end=image_point(camera,*result['center'][-1],w,h)
        if end:
            cv2.drawMarker(canvas,end,(0,0,255),cv2.MARKER_TILTED_CROSS,16,2)
            cv2.putText(canvas,'END',(end[0]+8,end[1]),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,255),1)
    header=np.zeros((185,w,3),dtype=np.uint8)
    visibility=describe_visibility(dict(result,image_size=[w,h]))
    lines=[f'VROOM STEP 1 v{result.get("algorithm_version",1)} | {result["status"]}',
           f'points={result["candidate_count"]} clusters={result["cluster_count"]} process={result["processing_ms"]:.1f}ms',
           result['reason'],
           'Tracking: '+result.get('tracking_state','OFFLINE_SINGLE_FRAME')+' | '+result.get('center_source','none'),
           f'Center support: {visibility["center_visibility"]} | visible {visibility["visible_center_length_m"]:.2f}m (not driving approval)',
           'Yellow:points Cyan:left Magenta:right Green:center Orange:estimated',
           ('V12 AUTO TRIAL | G:start WASD:takeover Space:stop R/T:record' if result.get('controller',{}).get('enabled') else 'MANUAL | WASD Space:brake P:photo R:record T:end Q:quit')]
    if result.get('controller',{}).get('mode')=='ADAS_ONLY':
        lines[0]='ADAS CAR | '+result['status']
        lines[-1]='DRIVER CONTROL | H:assist Space:brake R/T:record Q:quit'
    for i,line in enumerate(lines):cv2.putText(header,line,(6,20+i*25),cv2.FONT_HERSHEY_SIMPLEX,.40,(230,230,230),1)
    return np.vstack([header,canvas])
