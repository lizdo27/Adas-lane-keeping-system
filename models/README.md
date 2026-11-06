# Model AI cho ADAS xe thật

Đặt trọng số tại `models/best.pt` từ thư mục gốc repo. Chưa có trọng số được đưa lên GitHub.
Model pilot là YOLOv8n segmentation nhận diện `lane_marking` và `crosswalk`; tên và thứ tự class phải khớp bộ chạy trên xe thật.

Nếu export ONNX, đặt `models/best.onnx`. Cần bộ chạy hỗ trợ định dạng tương ứng; việc chép file chưa tự tích hợp model vào xe.

Kèm thông tin mỗi model: phiên bản, ngày train, tên/thứ tự class, dataset và cách chia train/val/test, kích thước ảnh, metrics, phiên bản thư viện, SHA256 và link tải.

*.pt/*.onnx đang bị .gitignore loại khỏi commit, dù .gitattributes đã cấu hình LFS. Đặt file để chạy cục bộ trước; khi phát hành, chọn Git LFS hoặc link tải riêng và cập nhật ignore có chủ đích.

Bộ chạy xe thật chưa được cung cấp trong bản repo này. Không coi việc lưu model là đã hoàn thiện triển khai trên Pi 5.

## Thử model hiện tại

Code offline đã có tại [src/perception](../src/perception/README.md). Hybrid nhận lớp theo tên; notebook pilot dùng `0: lane_marking`, `1: crosswalk`. Không đưa model SEAME cũ vào đánh giá như thể đó là model dữ liệu thật mới.

[Xem metrics và giới hạn model pilot](../docs/EXPERIMENTS.md). Khi thay trọng số, ghi lại SHA256, dataset và phiên bản thư viện; không mặc định metrics cũ áp dụng cho model mới.
