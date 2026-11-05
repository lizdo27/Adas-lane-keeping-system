"""AI semantic support + original-image stripe geometry for the v10 controller.

No AI-only synthetic boundaries or CV-only steering fallback. A small search
band permits imperfect masks; stripe width/contrast is measured on the original
camera image. Existing geometry resolves lane identity, not mask instance order.
"""
import time
import cv2
import numpy as np
from vroom_lane import detect


def marking_geometry(frame, lane, crossing, config):
    if lane.shape != frame.shape[:2] or crossing.shape != frame.shape[:2]:
        raise ValueError('Masks must match the camera image')
    radius = max(1, round(frame.shape[1] * 3 / 640))
    support = cv2.dilate((lane > 0).astype(np.uint8),
                         np.ones((2*radius+1, 2*radius+1), np.uint8))
    result = detect(frame, config, stripe_support=support, stripe_exclusion=crossing)
    result.update(perception_source='AI+CV lane_marking',
                  algorithm_version='adas-marking-hybrid-1',
                  fusion_mode='AI supported original-image stripes',
                  ai_lane_pixels=int(np.count_nonzero(lane)),
                  ai_crosswalk_pixels=int(np.count_nonzero(crossing)))
    if not lane.any():
        result['reason'] = 'AI found no lane marking: manual steering only'
    elif not result['boundaries']:
        result['reason'] = 'AI/CV geometry unresolved: ' + result['reason']
    return result


class MarkingPerception:
    def __init__(self, weights, conf=.25, device='cpu', imgsz=640):
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(2)
        cv2.setNumThreads(1)
        self.model = YOLO(str(weights))
        self.weights, self.conf, self.device, self.imgsz = str(weights), conf, device, imgsz
        if self.model.task != 'segment' or 'lane_marking' not in self.model.names.values():
            raise ValueError('Expected segmentation checkpoint with lane_marking class')
        self.classes = [int(k) for k, v in self.model.names.items()
                        if v in ('lane_marking', 'crosswalk')]
        # Warmup before any vehicle publisher exists.
        self.model.predict(np.zeros((480,640,3), np.uint8), imgsz=imgsz,
                           device=device, classes=self.classes, verbose=False)

    def detect(self, frame, config):
        started = time.perf_counter()
        h, w = frame.shape[:2]
        pred = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf,
                                  classes=self.classes, device=self.device,
                                  retina_masks=True, verbose=False)[0]
        inference_finished=time.perf_counter()
        lane = np.zeros((h,w), np.uint8)
        crossing = np.zeros_like(lane)
        counts = {'lane_marking': 0, 'crosswalk': 0}
        confidences = []
        if pred.masks is not None:
            for mask, cls, confidence in zip(pred.masks.data.cpu().numpy(),
                                            pred.boxes.cls.cpu().numpy(),
                                            pred.boxes.conf.cpu().numpy()):
                name = self.model.names[int(cls)]
                target = lane if name == 'lane_marking' else crossing
                mask = cv2.resize((mask>.5).astype(np.uint8), (w,h),
                                  interpolation=cv2.INTER_NEAREST)
                np.maximum(target, mask, out=target)
                counts[name] += 1
                if name == 'lane_marking':
                    confidences.append(float(confidence))
        result = marking_geometry(frame, lane, crossing, config)
        result.update(model_path=self.weights, model_class='lane_marking',
                      model_confidence=max(confidences) if confidences else None,
                      mask_count=counts['lane_marking'], crosswalk_count=counts['crosswalk'],
                      inference_imgsz=self.imgsz, inference_ms=(inference_finished-started)*1000,
                      geometry_and_masks_ms=(time.perf_counter()-inference_finished)*1000,
                      processing_ms=(time.perf_counter()-started)*1000)
        overlay = frame.copy()
        for mask, color in ((lane,(0,180,0)), (crossing,(180,0,180))):
            selected = mask > 0
            overlay[selected] = (overlay[selected]*.8 + np.asarray(color)*.2).astype(np.uint8)
        return result, overlay
