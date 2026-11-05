"""Observed lane_area mask geometry. No brightness/stripe detector fallback."""
from dataclasses import asdict
import time
import cv2
import numpy as np
from vroom_lane import fit_boundary, paired_center, describe_visibility, near_side, offset_boundary


def mask_geometry(mask, config):
    h,w=mask.shape; camera=config.camera()
    count,components,stats,_=cv2.connectedComponentsWithStats((mask>0).astype(np.uint8),8)
    # Select only an unambiguous component extending into the near camera view.
    # Detached far blobs must not invalidate a separately observed near lane.
    candidates=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=.01*w*h
                and np.any(components[int(.65*h):]==i)]
    chosen=components==candidates[0] if len(candidates)==1 else np.zeros_like(mask,bool)
    samples=[[],[]];pixels=[];closed=[False,False];ambiguous=False;limit=None
    border=max(2,int(.03*w)) # segmentation resize fringes at crop are not road edges
    for v in np.linspace(int(h*config.roi_bottom),int(h*config.roi_top),config.scan_rows).astype(int):
        try:x,_=camera.ground(w/2,float(v),w,h)
        except ValueError:continue
        if not config.near_m<=x<=config.far_m:continue
        row=np.flatnonzero(chosen[v])
        runs=np.split(row,np.flatnonzero(np.diff(row)>1)+1)
        runs=[r for r in runs if len(r)>=4]
        if len(runs)!=1:
            if any(samples):limit=x;break
            if len(runs)>1:ambiguous=True;break
            continue
        a,b=int(runs[0][0]),int(runs[0][-1])
        _,yl=camera.ground(a,float(v),w,h);_,yr=camera.ground(b,float(v),w,h)
        clipped=[a<border,b>=w-border]
        if not any(clipped) and not config.lane_min_m<=yl-yr<=config.lane_max_m:
            if any(samples):limit=x;break
            ambiguous=True;break
        for side,(u,y) in enumerate(((a,yl),(b,yr))):
            if clipped[side]:
                if samples[side]:closed[side]=True
            elif not closed[side]:
                samples[side].append([x,y]);pixels.append([u,int(v)])
    bounds=[]
    for i,points in enumerate(samples):
        if not points:continue
        boundary=fit_boundary(np.asarray(points),config,i,0)
        if boundary:bounds.append(dict(boundary,side=('left','right')[i]))
    center=[];width=None;status='NO_LANE';source='none';reason='AI: insufficient observed mask edges'
    if len(bounds)==2:
        center,width=paired_center(bounds[0],bounds[1],config)
        if len(center):
            status='TWO_BOUNDARIES';source='two_boundaries';reason='AI lane_area: paired observed mask edges'
    elif len(bounds)==1:
        b=bounds[0]
        # A component's image ordering is not enough once the vehicle crosses it.
        if near_side(b)==b['side']:
            center=offset_boundary(b,config.lane_width_m,b['side']).tolist()
            if center:
                status='ONE_BOUNDARY_ESTIMATE';source='configured_width'
                reason='AI: one observed edge; opposite edge cropped, center estimated'
        else:
            b['side']='unknown';status='UNCERTAIN_SIDE';reason='AI edge side uncertain'
    if ambiguous or len(candidates)>1:
        bounds=[];center=[];width=None;source='none';status='AMBIGUOUS_BRANCH';reason='AI: multiple near regions or ambiguous near mask'
    points=samples[0]+samples[1]
    return describe_visibility(dict(status=status,reason=reason,points=points,pixels=pixels,
        labels=[0]*len(points),boundaries=bounds,center=center,
        center_source=source,measured_lane_width_m=width,
        path_limit_m=limit,branch_groups=[],algorithm_version='ai-lane-area-2',
        ignored_far_components=max(0,count-1-len(candidates)),
        recovered_points=0,candidate_count=len(points),cluster_count=len(bounds),
        dense_markings=False,transverse_markings=[],pair_resolution=None,pair_candidates=[],
        processing_ms=0.,config=asdict(config),image_size=[w,h],perception_source='AI lane_area'))



class AILanePerception:
    def __init__(self,weights,conf=.5):
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(2)
        cv2.setNumThreads(1)
        self.model=YOLO(str(weights));self.weights=str(weights);self.conf=conf
        self.classes=[int(k) for k,v in self.model.names.items() if v=='lane_area']
        if not self.classes or self.model.task!='segment':
            raise ValueError('Model must be a segmentation model with class lane_area')
        self.model.predict(np.zeros((640,640,3),np.uint8),imgsz=640,device='cpu',verbose=False)

    def detect(self,frame,config):
        started=time.perf_counter();h,w=frame.shape[:2]
        pred=self.model.predict(cv2.resize(frame,(640,640)),imgsz=640,conf=self.conf,
            classes=self.classes,device='cpu',retina_masks=True,verbose=False)[0]
        mask=np.zeros((h,w),np.uint8);confidence=None
        count=0 if pred.masks is None else len(pred.masks.data)
        if count==1:
            mask=cv2.resize((pred.masks.data[0].cpu().numpy()>.5).astype(np.uint8),(w,h),interpolation=cv2.INTER_NEAREST)
            confidence=float(pred.boxes.conf[0].item())
        result=mask_geometry(mask,config)
        if count>1:
            result.update(status='AMBIGUOUS_BRANCH',reason='Multiple AI lane masks: lane identity unresolved')
        result.update(model_path=self.weights,model_class='lane_area',model_confidence=confidence,mask_count=count,
                      processing_ms=(time.perf_counter()-started)*1000)
        overlay=frame.copy();selected=mask.astype(bool)
        overlay[selected]=(overlay[selected]*.75+np.array([0,180,0])*.25).astype(np.uint8)
        return result,overlay
