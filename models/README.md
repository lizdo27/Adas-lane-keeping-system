# Model AI cho ADAS xe thật

Đã phát hành model pilot tại `models/best.pt` qua **Git LFS**. File này được chép nguyên byte từ `best_real.pt` đã dùng thử video thật, không phải model SEAME cũ.

- Kiến trúc xác nhận khi mở file: YOLOv8n-seg, task segmentation.
- Hai lớp: `0: lane_marking`, `1: crosswalk`.
- Kích thước: 7.014.516 byte.
- SHA256 và metadata: [model_metadata.json](model_metadata.json).
- Kết quả validation và giới hạn: [EXPERIMENTS](../docs/EXPERIMENTS.md).
- Trạng thái: pilot; chưa đủ bằng chứng triển khai điều khiển xe thật.

## Tải đúng trọng số

Cài Git LFS trên máy, sau đó tại repo:

```bash
git lfs install
git lfs pull
sha256sum models/best.pt
```

Nếu file tải về chỉ khoảng vài trăm byte và mở ra thấy `version https://git-lfs.github.com/spec/v1`, đó là pointer, chưa phải trọng số. Chạy `git lfs pull`. Khi clone mới cũng cần Git LFS; tải ZIP repository có thể không chứa nội dung LFS tùy cấu hình GitHub.

## Chạy thử

```bash
python src/perception/hybrid_video.py --video /duong/dan/video_goc.mp4
```

Đọc [hướng dẫn perception](../src/perception/README.md) để cài môi trường. Model đã được kiểm tra mở và suy luận CPU trên ảnh tổng hợp; kiểm tra này xác nhận file chạy được, không đo độ chính xác.

Chép model chưa tự tích hợp vào ROS 2 hoặc xe thật. Nếu sau này export ONNX, cần kiểm tra output và bộ chạy tương ứng trước khi sử dụng. Các trọng số khác vẫn bị ignore để tránh vô tình phát hành model cũ.

Giấy phép repo không thay thế điều kiện sử dụng Ultralytics, checkpoint nền hoặc dữ liệu huấn luyện. Khi phát hành phiên bản mới, cập nhật SHA256, metadata và kết quả riêng cho phiên bản đó.
