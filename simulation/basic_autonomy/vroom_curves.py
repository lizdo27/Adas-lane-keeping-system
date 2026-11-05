"""Observed local curves and corner splitting; metres in the ground plane."""
from functools import lru_cache
import math
import numpy as np


def cut_corners(points):
    """Split abrupt tangent changes, not the gradual rotation of a bend."""
    if len(points)<9:return [points],[]
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    candidates=[]
    for i in range(3,len(points)-3):
        left=np.flatnonzero(arc[:i]<=arc[i]-.035)
        right=np.flatnonzero(arc[i+1:]>=arc[i]+.035)+i+1
        if not len(left) or not len(right):continue
        a=int(left[-1]);b=int(right[0])
        if arc[i]-arc[a]>.12 or arc[b]-arc[i]>.12:continue
        v=points[i]-points[a];w=points[b]-points[i]
        angle=abs(math.atan2(np.cross(v,w),np.dot(v,w)))
        if angle<math.radians(35):continue
        steps=np.diff(points[max(0,i-1):i+3],axis=0)
        turns=[abs(math.atan2(np.cross(u,v),np.dot(u,v))) for u,v in zip(steps[:-1],steps[1:])]
        if max(turns)<math.radians(25):continue
        # Both legs must be approximately straight around an actual corner.
        errors=[]
        for segment in (points[a:i+1],points[i:b+1]):
            chord=segment[-1]-segment[0]
            errors.append(float(np.max(np.abs(np.cross(segment-segment[0],chord)))/np.linalg.norm(chord)))
        if max(errors)>.006:continue
        candidates.append((angle,i))
    cuts=[]
    for _,i in sorted(candidates,reverse=True):
        if all(abs(i-j)>5 for j in cuts):cuts.append(i)
    cuts.sort()
    bounds=[0]+cuts+[len(points)-1]
    return [points[a:b+1] for a,b in zip(bounds[:-1],bounds[1:])],[float(points[i,0]) for i in cuts]


def smooth_track(points):
    """Local quadratic smoothing on actual support; no global bend extrapolation."""
    result=points.copy()
    if not len(points):return result
    # Same seven-point windows and column-scaled least squares as polyfit,
    # evaluated in one batch instead of hundreds of Python/LAPACK calls.
    count=min(7,len(points))
    starts=np.maximum(0,np.minimum(np.arange(len(points))-3,len(points)-7))
    parts=points[starts[:,None]+np.arange(count)]
    scale=np.maximum(np.ptp(parts[:,:,0],axis=1),1e-6)
    x=(parts[:,:,0]-points[:,0,None])/scale[:,None]
    degree=min(2,count-1)
    design=x[:,:,None]**np.arange(degree,-1,-1)
    norms=np.sqrt(np.sum(design**2,axis=1))
    # Preserve polyfit's singular-window fallback rather than invert A.T @ A.
    if np.any(norms==0):
        for i,part in enumerate(parts):
            result[i,1]=np.polyfit(x[i],part[:,1],degree)[-1]
        return result
    inverse=np.linalg.pinv(design/norms[:,None,:],rcond=count*np.finfo(float).eps)
    coeff=np.einsum('nik,nk->ni',inverse,parts[:,:,1])/norms
    result[:,1]=coeff[:,-1]
    return result


def short_branch_contact(tracks):
    """A short transverse joining mark must not disappear under span filtering."""
    tracks=[t for t in tracks if len(t)>=6 and np.linalg.norm(np.ptp(t,axis=0))>=.10]
    contacts=[]
    for i,a in enumerate(tracks):
        for b in tracks[i+1:]:
            spans=[np.linalg.norm(np.ptp(t,axis=0)) for t in (a,b)]
            if min(spans)>=.16 or max(spans)<.16:continue
            d=np.linalg.norm(a[:,None,:]-b[None,:,:],axis=2)
            ai,bi=np.unravel_index(np.argmin(d),d.shape)
            if d[ai,bi]>.06:continue
            tangents=[]
            for points,j in ((a,ai),(b,bi)):
                tangents.append(points[min(len(points)-1,j+3)]-points[max(0,j-3)])
            u,v=tangents;angle=abs(math.atan2(np.cross(u,v),np.dot(u,v)))
            if angle>math.radians(35):contacts.append(float(min(a[ai,0],b[bi,0])))
    return min(contacts) if contacts else None


def curve_y(b,x):
    if 'path' in b:
        p=np.asarray(b['path']);return np.interp(x,p[:,0],p[:,1])
    return np.polyval(b['coeff'],x)


@lru_cache(maxsize=64)
def _path_slopes(path):
    """Memoize only pure geometry, keyed by ALL immutable point coordinates.

    No image, lane identity or timestamp cache. Changed points cannot reuse the
    old derivatives. Return tuples so callers cannot corrupt cached values.
    """
    p=np.asarray(path);d=[]
    for px,py in p:
        ids=np.flatnonzero(np.abs(p[:,0]-px)<=.06)
        if len(ids)<5:ids=np.argsort(np.abs(p[:,0]-px))[:min(5,len(p))]
        part=p[ids];scale=max(float(np.ptp(part[:,0])),1e-6)
        coeff=np.polyfit((part[:,0]-px)/scale,part[:,1],min(2,len(part)-1))
        d.append(float(coeff[-2]/scale))
    return tuple(d)


def curve_slope(b,x):
    if 'path' in b:
        p=np.asarray(b['path'])
        d=_path_slopes(tuple(map(tuple,b['path'])))
        return np.interp(x,p[:,0],d)
    return np.polyval(np.polyder(b['coeff']),x)


def curve_points(b,count=60):
    xs=np.linspace(b['lo'],b['hi'],count)
    return np.column_stack([xs,curve_y(b,xs)])


def match_curves(left,right,c):
    """Intersect local normals with observed polyline segments, in either direction."""
    centers=[];widths=[]
    for source,target,direction in ((left,right,-1),(right,left,1)):
        source_xy=curve_points(source,60);target_xy=curve_points(target,100)
        q=target_xy[:-1];v=np.diff(target_xy,axis=0)
        vl=np.linalg.norm(v,axis=1)
        slopes=curve_slope(source,source_xy[:,0])
        tangents=np.column_stack([np.ones(len(slopes)),slopes])/np.hypot(1.,slopes)[:,None]
        normals=direction*np.column_stack([-tangents[:,1],tangents[:,0]])
        den=normals[:,0,None]*v[None,:,1]-normals[:,1,None]*v[None,:,0]
        valid=np.abs(den)>1e-10;safe=np.where(valid,den,1.)
        delta=q[None,:,:]-source_xy[:,None,:]
        distance=(delta[:,:,0]*v[None,:,1]-delta[:,:,1]*v[None,:,0])/safe
        t=(delta[:,:,0]*normals[:,1,None]-delta[:,:,1]*normals[:,0,None])/safe
        valid&=(distance>=c.lane_min_m)&(distance<=c.lane_max_m)&(t>=0)&(t<=1)
        valid&=((tangents@v.T)/np.maximum(vl,1e-10))>=math.cos(math.radians(30))
        supported=np.any(valid,axis=1)
        distances=np.min(np.where(valid,distance,np.inf),axis=1)[supported]
        centers.extend((source_xy[supported]+normals[supported]*distances[:,None]/2).tolist())
        widths.extend(distances.tolist())
    if len(centers)<8 or np.std(widths)>.07:return [],None
    xy=np.array(centers);xy=xy[np.argsort(xy[:,0])]
    # Merge repeated x values before local interpolation (e.g. straight lanes).
    xs,ids=np.unique(np.round(xy[:,0],7),return_inverse=True)
    ys=np.bincount(ids,weights=xy[:,1])/np.bincount(ids)
    if len(xs)<8 or np.ptp(xs)<.03:return [],None
    if np.linalg.norm(xy[-1]-xy[0])<c.min_span_m:return [],None
    # Samples from opposing normals must agree, not create a zig-zag average.
    smooth=smooth_track(np.column_stack([xs,ys]))
    if np.sqrt(np.mean((ys-smooth[:,1])**2))>.025:return [],None
    xx=np.linspace(xs.min(),xs.max(),50)
    return np.column_stack([xx,np.interp(xx,smooth[:,0],smooth[:,1])]).tolist(),float(np.median(widths))
