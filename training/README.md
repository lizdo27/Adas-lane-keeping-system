# Huấn luyện YOLOv8n-seg trên Colab

Mở [Train_ADAS_REAL.ipynb](Train_ADAS_REAL.ipynb) trong Colab. Upload `ADAS_REAL_MERGED.zip` vào gốc My Drive, bật T4 GPU và chạy các cell theo thứ tự.

Notebook kiểm tra class/split/nhãn, train, lưu checkpoint lên Drive, đánh giá test và tải best.pt. Nó yêu cầu ZIP đã hợp nhất có `data.yaml`, `manifest.json`, `merge_report.json`, `train/`, `valid/`, `test/`; không dùng trực tiếp cho một ZIP Roboflow bất kỳ.

Bản lưu trong repo đã xóa output và cố định Ultralytics 8.4.202 theo log pilot. Notebook ban đầu cài phiên bản mới nhất tại thời điểm chạy. Python/Torch/CUDA phụ thuộc runtime Colab; `environment.txt` và SHA256 dataset được lưu cùng run để đối chiếu.

Thông số pilot: YOLOv8n-seg pretrained, imgsz 640, batch 8, epochs 100, patience 20, seed 42. Các augmentation được liệt kê trong lệnh train đều đặt 0 vì ZIP đã chứa biến thể. Không cần train đủ 100 epoch khi early stopping được kích hoạt. Không mặc định tăng ảnh bằng nhân bản sẽ tăng chất lượng.

Dùng validation để chọn model. Cell đánh giá test chỉ chạy sau khi chốt cấu hình; không tối ưu liên tục theo test. Nếu mất kết nối giữa lượt train, làm theo cell resume trong notebook. Fine-tune dữ liệu mới là lượt train riêng từ best.pt, không phải resume lượt cũ.

Sau train chép `best.pt` vào `models/best.pt` và thử video gốc theo [hướng dẫn perception](../src/perception/README.md). Model không tự được tích hợp vào ROS 2 hoặc bộ điều khiển xe.
