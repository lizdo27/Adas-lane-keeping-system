# Model AI cho ADAS xe thật

Đặt trọng số tại `models/best.pt` từ thư mục gốc repo. Chưa có trọng số được đưa lên GitHub.
Model dự kiến là YOLO segmentation nhận diện `lane_marking` và `crosswalk`; tên và thứ tự class phải khớp bộ chạy trên xe thật.

Nếu export ONNX, đặt `models/best.onnx`. Cần bộ chạy hỗ trợ định dạng tương ứng; việc chép file chưa tự tích hợp model vào xe.

Kèm thông tin mỗi model: phiên bản, ngày train, tên/thứ tự class, dataset và cách chia train/val/test, kích thước ảnh, metrics, phiên bản thư viện, SHA256 và link tải.

*.pt/*.onnx đang bị .gitignore loại khỏi commit, dù .gitattributes đã cấu hình LFS. Đặt file để chạy cục bộ trước; khi phát hành, chọn Git LFS hoặc link tải riêng và cập nhật ignore có chủ đích.

Bộ chạy xe thật chưa được cung cấp trong bản repo này. Không coi việc lưu model là đã hoàn thiện triển khai trên Pi 5.
