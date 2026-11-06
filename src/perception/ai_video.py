"""Test segmentation on a recorded video; never sends vehicle commands."""
import argparse
import math
import time
from pathlib import Path


def main():
    import cv2
    from ultralytics import YOLO

    base = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, default=base / 'models/best.pt')
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--no-show', action='store_true', help='Save video without opening a window')
    args = parser.parse_args()
    if not 0 < args.conf < 1:
        parser.error('--conf must be between 0 and 1')
    for path in (args.weights, args.video):
        if not path.is_file():
            parser.error(f'Không thấy file: {path}')

    model = YOLO(str(args.weights))
    if model.task != 'segment':
        parser.error('Cần model segmentation.')
    print('Model:', args.weights, flush=True)
    print('Các lớp:', model.names, flush=True)
    cap = cv2.VideoCapture(str(args.video))
    writer = None
    frames = lane_frames = crosswalk_frames = 0
    times = []
    window = 'AI test - Q: an xem truoc, van luu video'
    show_preview = not args.no_show
    reached_end = False
    interrupted = False
    total_frames = 0
    output = args.video.with_name(args.video.stem + '_ai_test_' + time.strftime('%Y%m%d_%H%M%S') + '.mp4')
    try:
        if not cap.isOpened():
            raise RuntimeError('Không mở được video.')
        fps = cap.get(cv2.CAP_PROP_FPS)
        if not math.isfinite(fps) or fps <= 0:
            fps = 30.0
            print('Không đọc được FPS nguồn; dùng 30 FPS.')
        reported_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        total_frames = int(reported_frames) if math.isfinite(reported_frames) and reported_frames > 0 else 0
        print(f'Video nguồn: {total_frames} khung, khoảng {total_frames / fps:.1f} giây.', flush=True)
        print('Đang xử lý toàn bộ video. Q/đóng cửa sổ chỉ ẩn xem trước; Ctrl+C mới dừng.', flush=True)
        if output.exists():
            raise FileExistsError(output)
        if not args.no_show:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 960, 540)
        while True:
            ok, frame = cap.read()
            if not ok:
                reached_end = True
                break
            if writer is None:
                height, width = frame.shape[:2]
                writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
                if not writer.isOpened():
                    raise RuntimeError('Không tạo được video kết quả.')
            started = time.perf_counter()
            result = model.predict(frame, imgsz=640, conf=args.conf, device=args.device,
                                   retina_masks=True, verbose=False)[0]
            elapsed = time.perf_counter() - started
            classes = result.boxes.cls.cpu().tolist() if result.boxes is not None else []
            detected = {model.names[int(c)] for c in classes}
            frames += 1
            lane_frames += int('lane_marking' in detected)
            crosswalk_frames += int('crosswalk' in detected)
            if frames > 1:
                times.append(elapsed)
            overlay = result.plot()
            cv2.putText(overlay, f'Frame {frames} | Inference {elapsed * 1000:.0f} ms',
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            writer.write(overlay)
            if frames % 100 == 0:
                progress = f'{frames}/{total_frames} ({100 * frames / total_frames:.1f}%)' if total_frames else str(frames)
                print(f'Đã lưu {progress} khung | {frames / fps:.1f} giây video nguồn', flush=True)
            if show_preview:
                cv2.imshow(window, overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    show_preview = False
                    cv2.destroyAllWindows()
                    print('Đã ẩn xem trước; vẫn tiếp tục xử lý và lưu hết video.', flush=True)
    except KeyboardInterrupt:
        interrupted = True
        print('\nĐã dừng theo yêu cầu; video kết quả chỉ là một phần.')
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if not args.no_show:
            cv2.destroyAllWindows()
    print(f'\nĐã xử lý: {frames} khung hình')
    if reached_end and total_frames and frames < total_frames:
        print(f'CẢNH BÁO: giải mã dừng sớm ({frames}/{total_frames} khung). Kết quả chưa đủ video nguồn.')
    elif reached_end and not interrupted:
        print('Đã xử lý đến cuối luồng video đọc được.')
    if frames:
        print('Thời lượng video kết quả:', round(frames / fps, 1), 'giây')
        print(f'Có lane_marking: {lane_frames / frames:.1%} khung')
        print(f'Có crosswalk: {crosswalk_frames / frames:.1%} khung')
        print('Video kết quả:', output)
    else:
        print('Không đọc được khung hình nào; chưa có video kết quả dùng được.')
    if times:
        mean = sum(times) / len(times)
        print(f'Thời gian suy luận trung bình: {mean * 1000:.1f} ms; tương đương {1 / mean:.1f} FPS')
    print('Tỷ lệ xuất hiện nhãn KHÔNG phải độ chính xác. Video lưu theo FPS nguồn.')


if __name__ == '__main__':
    main()
