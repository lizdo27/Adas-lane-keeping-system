# Trọng số AI

Đặt model thật sau này tại `models/best.pt` (tính từ thư mục gốc repo).
Chưa có trọng số được đưa vào repo. Chế độ `geometry` hiện tại không sử dụng model.

## Khi thử chế độ AI + OpenCV hiện có

Model cần là YOLO segmentation, có class `lane_marking`; `crosswalk` dùng để loại vùng vạch qua đường. Model lane_area cũ không thay thế trực tiếp được.

Từ thư mục `simulation/`, sau khi cài Ultralytics/PyTorch tương thích trong môi trường Python:

```bash
source ./bashr
basic_autonomy/.venv/bin/python basic_autonomy/adas_view.py --perception hybrid --weights ../models/best.pt --speed 0.06
```

Phải truyền `--weights`: chương trình chưa mặc định tìm ở models/ gốc repo. Đây là chế độ hybrid thử nghiệm, cần kiểm tra độ trễ và nhận diện trước khi dùng; không phải thuật toán kết hợp mới đã được kiểm chứng.

Nếu export ONNX, đặt `models/best.onnx`; chỉ lưu file không đồng nghĩa bộ chạy hiện tại đã hỗ trợ nó.

Kèm thông tin mỗi model: phiên bản, ngày train, tên/thứ tự class, dataset và cách chia train/val/test, kích thước ảnh, metrics, phiên bản thư viện, SHA256 và link tải.

*.pt/*.onnx đang bị .gitignore loại khỏi commit, dù .gitattributes đã có cấu hình LFS. Đặt file ở đây để chạy cục bộ trước; khi cần phát hành trọng số, chọn Git LFS hoặc link tải riêng và cập nhật quy tắc ignore có chủ đích.
