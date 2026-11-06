"""Offline AI + CV lane experiment. No ROS or vehicle commands.
Green mask: fused paint evidence. No fitted curves or vehicle control.
"""
import argparse
import json
import math
import time
from pathlib import Path
import cv2
import numpy as np


def components(mask):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        if area >= 8:
            yield labels == index, (int(x), int(y), int(width), int(height), int(area))


def cv_mask(frame, top):
    """Current-frame paint candidates: bright on BOTH sides, bounded stripe width.

    Never close gaps between dashes or fill a fitted lane corridor.
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    smooth = cv2.GaussianBlur(gray, (3, 3), 0).astype(np.float32)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    contrast = np.zeros((h, w), np.float32)
    # Horizontal cross-sections must be brighter than two side backgrounds.
    for radius in (3, 5, 8, 12, 18, 25):
        left = np.roll(smooth, radius, axis=1)
        right = np.roll(smooth, -radius, axis=1)
        ridge = np.minimum(smooth - left, smooth - right)
        ridge[:, :radius] = 0
        ridge[:, -radius:] = 0
        contrast = np.maximum(contrast, ridge)
    raw = ((contrast > 14) & (hsv[:, :, 1] < 90) & (gray > 95)).astype(np.uint8)
    raw[:top] = 0
    raw[:, :5] = 0
    raw[:, -5:] = 0
    raw = cv2.morphologyEx(raw, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    filtered = np.zeros_like(raw)
    strict = np.zeros_like(raw)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(raw, 8)
    for index in range(1, count):
        x, y, width, height, area = map(int, stats[index])
        if area < 14 or height < 5 or height < .2 * width:
            continue
        region = labels[y:y+height, x:x+width] == index
        yy, xx = np.nonzero(region)
        points = np.column_stack((xx, yy)).astype(np.float32)
        rw, rh = cv2.minAreaRect(points)[1]
        length, thickness = max(rw, rh), max(1., min(rw, rh))
        if length / thickness < 2.0 or thickness > w * .055:
            continue
        filtered[y:y+height, x:x+width][region] = 1
        hull_area = cv2.contourArea(cv2.convexHull(points))
        solidity = area / max(area, hull_area)
        occupancy = area / max(area, rw * rh)
        if (length / thickness >= 2.3 and height >= 10 and
                solidity >= .90 and occupancy >= .75 and
                float(np.median(contrast[y:y+height, x:x+width][region])) > 19):
            strict[y:y+height, x:x+width][region] = 1
    return filtered, strict, gray


def ai_masks(result, shape):
    lane = np.zeros(shape, np.float32)
    crossing = np.zeros(shape, np.float32)
    if result.masks is None or result.boxes is None:
        return lane, crossing
    for mask, cls, confidence in zip(result.masks.data.cpu().numpy(), result.boxes.cls.cpu().numpy(), result.boxes.conf.cpu().numpy()):
        name = result.names[int(cls)]
        target = lane if name == 'lane_marking' else crossing if name in ('crosswalk', 'crosswalks') else None
        if target is not None:
            mask = cv2.resize(mask, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
            np.maximum(target, (mask > .5) * float(confidence), out=target)
    return lane, crossing


def paint(frame, mask, color):
    out = frame.copy()
    valid = mask > 0
    out[valid] = (out[valid] * .65 + np.array(color) * .35).astype(np.uint8)
    contours, _ = cv2.findContours(valid.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, contours, -1, color, 1)
    return out


def stripe_anchors(mask):
    """Internal local direction tests, never rendered or used to fill gaps."""
    h, w = mask.shape
    anchors = []
    for region, (x, y, width, height, area) in components(mask):
        if height < 12 or area < 45:
            continue
        yy, xx = np.nonzero(region)
        samples_y = np.arange(y, y + height, max(1, height // 30))
        samples_x = []
        good_y = []
        for ry in samples_y:
            pixels = np.flatnonzero(region[ry])
            if len(pixels):
                good_y.append(ry)
                samples_x.append(float(np.median(pixels)))
        if len(good_y) < 5:
            continue
        degree = 2 if height > .2 * h else 1
        coef = np.polyfit(np.array(good_y) / h, np.array(samples_x) / w, degree)
        residual = np.median(abs(np.polyval(coef, np.array(good_y) / h) * w - samples_x))
        if residual <= 8:
            anchors.append((coef, y, y + height))
    return anchors


def aligned_with_anchor(region, anchors):
    h, w = region.shape
    yy, xx = np.nonzero(region)
    if len(xx) < 8:
        return False
    vx, vy, _, _ = cv2.fitLine(np.column_stack((xx, yy)).astype(np.float32), cv2.DIST_L2, 0, .01, .01).ravel()
    cy = float(np.median(yy))
    for coef, low, high in anchors:
        if cy < low - .18 * h or cy > high + .18 * h:
            continue
        expected = np.polyval(coef, yy / h) * w
        # A narrow support corridor admits missing paint, not nearby glare.
        if np.median(abs(xx - expected)) > max(7, .018 * w):
            continue
        slope = float(np.polyval(np.polyder(coef), cy / h)) * w / h
        direction = np.array([slope, 1.])
        direction /= np.linalg.norm(direction)
        if abs(float(direction @ np.array([vx, vy]))) >= .92:
            return True
    return False


def dash_chain_regions(strict):
    """Independent CV evidence: >=3 separate, aligned compact paint strips."""
    items = []
    h, w = strict.shape
    for region, (x, y, width, height, area) in components(strict):
        if height < 8 or area < 30:
            continue
        yy, xx = np.nonzero(region)
        pts = np.column_stack((xx, yy)).astype(np.float32)
        vx, vy, _, _ = cv2.fitLine(pts, cv2.DIST_L2, 0, .01, .01).ravel()
        items.append((region, np.array([xx.mean(), yy.mean()]), np.array([vx, vy])))
    result = np.zeros_like(strict)
    for region, center, direction in items:
        neighbors = 0
        for other, c2, d2 in items:
            delta = c2 - center
            if np.linalg.norm(delta) < 5:
                continue
            along = abs(float(delta @ direction))
            perpendicular = abs(float(delta[0] * direction[1] - delta[1] * direction[0]))
            if (along < .28 * h and perpendicular < 7 and
                    abs(float(direction @ d2)) > .96):
                neighbors += 1
        if neighbors >= 2:
            result[region] = 1
    return result


class Fusion:
    """Pixel-mask evidence fusion. Not ego-lane selection or a steering algorithm."""
    def __init__(self):
        self.previous_gray = None
        self.previous_hits = None
        self.previous_mask = None
        self.previous_stamp = None
        self.previous_anchor = None
        self.previous_anchor_age = None

    def process(self, frame, lane, cross, top, stamp):
        h, w = frame.shape[:2]
        cv, strict, gray = cv_mask(frame, top)
        ai = lane.copy()
        ai[:top] = 0
        warped_mask = np.zeros((h, w), np.uint8)
        warped_hits = np.zeros((h, w), np.float32)
        warped_anchor = np.zeros((h, w), np.uint8)
        anchor_age = np.ones((h, w), np.float32)
        if (self.previous_gray is not None and self.previous_gray.shape == gray.shape and
                0 < stamp - self.previous_stamp <= .2):
            small_size = (max(16, w // 2), max(16, h // 2))
            current = cv2.resize(gray, small_size)
            previous = cv2.resize(self.previous_gray, small_size)
            # Backward flow maps current pixels to previous coordinates for remap.
            flow = cv2.calcOpticalFlowFarneback(current, previous, None, .5, 3, 21, 3, 5, 1.2, 0)
            flow = cv2.resize(flow, (w, h))
            flow[:, :, 0] *= w / small_size[0]
            flow[:, :, 1] *= h / small_size[1]
            xx, yy = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
            mx, my = xx + flow[:, :, 0], yy + flow[:, :, 1]
            warped_mask = cv2.remap(self.previous_mask, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
            warped_hits = cv2.remap(self.previous_hits, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
            previous_aligned = cv2.remap(self.previous_gray, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            trustworthy = cv2.absdiff(gray, previous_aligned) < 35
            warped_anchor = cv2.remap(self.previous_anchor, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
            anchor_age = cv2.remap(self.previous_anchor_age, mx, my, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=1)
            anchor_age += stamp - self.previous_stamp
            warped_anchor[(~trustworthy) | (anchor_age > .25)] = 0
            warped_mask[~trustworthy] = 0
            warped_hits[~trustworthy] = 0
        # Temporal evidence must coincide with CURRENT CV paint, never paint old pixels.
        hits = np.where(strict > 0, np.minimum(warped_hits + 1, 5), 0).astype(np.float32)
        accepted_ai = np.zeros((h, w), np.uint8)
        cv_near = cv2.dilate(cv, np.ones((5, 5), np.uint8)) > 0
        retained = rejected = refined = 0
        for region, (x, y, width, height, area) in components(ai > 0):
            support = float(cv_near[region].mean())
            history = float((warped_mask[region] > 0).mean())
            confidence = float(np.max(ai[region]))
            # Do not turn a large floor-shaped hallucination into a lane mask.
            shape_ok = area / max(1, height) < w * .10 and height >= 5
            if shape_ok and (confidence >= .35 or support >= .25 or history >= .4):
                # Only refine locally if CV explains most of this actual mask.
                # At blur/weak CV, preserve AI instead of erasing it.
                if support >= .80:
                    accepted_ai[region & cv_near] = 1
                    refined += 1
                else:
                    accepted_ai[region] = 1
                retained += 1
            else:
                rejected += 1
        near_ai = cv2.dilate(accepted_ai, np.ones((17, 17), np.uint8)) > 0
        # Anchors are fresh AI observations, never self-reinforced CV additions.
        anchor_mask = accepted_ai | warped_anchor
        anchor_age[accepted_ai > 0] = 0
        anchors = stripe_anchors(anchor_mask)
        chain = dash_chain_regions(strict)
        added = np.zeros_like(cv)
        rejected_unaligned = 0
        for region, (_, _, _, _, area) in components(cv):
            overlap = float(near_ai[region].mean())
            persistent = float((hits[region] >= 3).mean())
            cross_overlap = float((cross[region] >= .55).mean())
            if cross_overlap > .5 and overlap < .1:
                continue
            if overlap >= .2:
                # Bound expansions to 8 px from existing AI; cannot flood glare component.
                added[region & near_ai] = 1
            elif persistent >= .65:
                chain_fraction = float((chain[region] > 0).mean())
                if aligned_with_anchor(region, anchors) or chain_fraction >= .65:
                    added[region & (strict > 0)] = 1
                else:
                    rejected_unaligned += 1
            elif overlap < .2 and aligned_with_anchor(region, anchors):
                # Recover an AI gap only using current, strict paint evidence.
                added[region & (strict > 0)] = 1
        fused = accepted_ai | added
        fused[:top] = 0
        self.previous_anchor = anchor_mask
        self.previous_anchor_age = anchor_age
        self.previous_gray = gray.copy()
        self.previous_mask = fused.copy()
        self.previous_hits = hits
        self.previous_stamp = stamp
        info = dict(ai_components_kept=retained, ai_components_rejected=rejected,
                    ai_components_refined=refined, cv_candidate_pixels=int(cv.sum()),
                    ai_retained_pixels=int(accepted_ai.sum()),
                    cv_added_pixels=int(((added > 0) & (accepted_ai == 0)).sum()),
                    fused_pixels=int(fused.sum()), cv_unaligned_rejected=rejected_unaligned, algorithm='paint-mask-v3')
        return cv, fused, info


def display_panel(frame, title):
    image = cv2.resize(frame, (576, 320))
    image = cv2.copyMakeBorder(image, 32, 0, 0, 0, cv2.BORDER_CONSTANT)
    cv2.putText(image, title, (8, 22), 0, .55, (255, 255, 255), 1)
    return image


def main():
    p = argparse.ArgumentParser(description=__doc__)
    base = Path(__file__).resolve().parents[2]
    p.add_argument('--weights', type=Path, default=base / 'models/best.pt')
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--conf', type=float, default=.25)
    p.add_argument('--device', default='cpu')
    p.add_argument('--roi-top', type=float, default=.40, help='Top of road ROI as image-height fraction')
    p.add_argument('--no-show', action='store_true')
    a = p.parse_args()
    if not a.weights.is_file() or not a.video.is_file(): p.error('Không thấy model/video; kiểm tra --weights và --video.')
    if not 0 < a.conf < 1 or not .1 <= a.roi_top <= .8: p.error('Invalid conf/roi-top')
    from ultralytics import YOLO
    model = YOLO(str(a.weights))
    if model.task != 'segment' or 'lane_marking' not in model.names.values(): p.error('Cần model segment có lane_marking.')
    cv2.setNumThreads(1)
    cap = cv2.VideoCapture(str(a.video))
    if not cap.isOpened(): raise RuntimeError('Không mở được video')
    fps = cap.get(cv2.CAP_PROP_FPS)
    fps = fps if math.isfinite(fps) and fps > 0 else 30.
    total = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    total = int(total) if math.isfinite(total) and total > 0 else 0
    out = a.video.parent / ('hybrid_masks_v3_' + time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(exist_ok=False)
    writers = []
    fusion = Fusion()
    show = not a.no_show
    n = 0
    completed = False
    timing = []
    print('Kết quả:', out, flush=True)
    print('Chạy toàn bộ video; Q chỉ ẩn cửa sổ. Ctrl+C mới dừng.', flush=True)
    try:
        if show:
            cv2.namedWindow('Hybrid comparison', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Hybrid comparison', 1152, 704)
        with (out / 'frames.jsonl').open('w') as log:
            while True:
                ok, frame = cap.read()
                if not ok:
                    completed = total == 0 or n >= total
                    break
                h, w = frame.shape[:2]
                if not writers:
                    for name, size in [('hybrid_full.mp4', (w, h)), ('comparison_full.mp4', (1152, 704))]:
                        writer = cv2.VideoWriter(str(out / name), cv2.VideoWriter_fourcc(*'mp4v'), fps, size)
                        writers.append(writer)
                        if not writer.isOpened(): raise RuntimeError('Không tạo được output video')
                t = time.perf_counter()
                result = model.predict(frame, imgsz=640, conf=a.conf, device=a.device, retina_masks=True, verbose=False)[0]
                lane, cross = ai_masks(result, (h, w))
                ai_ms = (time.perf_counter() - t) * 1000
                t = time.perf_counter()
                top = round(h * a.roi_top)
                cv, fused, info = fusion.process(frame, lane, cross, top, n / fps)
                fusion_ms = (time.perf_counter() - t) * 1000
                overlay = paint(frame, fused, (50, 230, 70))
                status = f'Mask v3 | AI {ai_ms:.0f} ms + CV/fusion {fusion_ms:.0f} ms'
                cv2.putText(overlay, status, (10, 25), 0, .5, (0, 255, 255), 1)
                cv2.putText(overlay, 'Green: paint mask | No fitted lane curves', (10, h - 12), 0, .4, (0, 255, 255), 1)
                ai_roi = lane.copy()
                ai_roi[:top] = 0
                ai_view = paint(frame, ai_roi, (255, 150, 40))
                cv_view = paint(frame, cv, (0, 255, 255))
                grid = np.vstack((np.hstack((display_panel(frame, 'SOURCE'), display_panel(ai_view, 'AI LANE MASK - same road ROI'))),
                                  np.hstack((display_panel(cv_view, 'CV PAINT CANDIDATES'), display_panel(overlay, 'HYBRID PAINT MASK - experimental')))))
                writers[0].write(overlay)
                writers[1].write(grid)
                log.write(json.dumps(dict(frame=n, time_s=n / fps, ai_ms=ai_ms, cv_fusion_ms=fusion_ms,
                                          fusion=info)) + '\n')
                n += 1
                if n > 1: timing.append((ai_ms, fusion_ms))
                if n % 100 == 0: print(f'Đã lưu {n}/{total} khung', flush=True)
                if show:
                    cv2.imshow('Hybrid comparison', grid)
                    if cv2.waitKey(1) & 0xff == ord('q') or cv2.getWindowProperty('Hybrid comparison', cv2.WND_PROP_VISIBLE) < 1:
                        show = False
                        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        print('Đã dừng: kết quả mới là một phần video.')
    finally:
        cap.release()
        for writer in writers: writer.release()
        if not a.no_show: cv2.destroyAllWindows()
        summary = dict(frames=n, source_frames=total, completed=completed, video=str(a.video), weights=str(a.weights),
                       roi_top=a.roi_top, conf=a.conf, fps_source=fps,
                       mean_ai_cv_ms=np.mean(timing, axis=0).tolist() if timing else [],
                       warning='Paint-mask fusion v3, not confirmed ego-lane boundaries. No ground truth accuracy or vehicle control. AI panel has same road ROI. CV fallback uses current paint and alignment to fresh AI anchors or a persistent multi-dash chain; anchors expire after 0.25 source seconds. Crosswalk mask used internally, not shown as lane.')
        (out / 'summary.json').write_text(json.dumps(summary, indent=2))
    print('Đã xử lý hết video.' if completed else 'CHƯA đủ video nguồn. Kiểm tra log/lỗi giải mã.')
    print('Kết quả:', out)


if __name__ == '__main__': main()
