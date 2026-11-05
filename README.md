# ADAS Lane Keeping System

Dự án ADAS hỗ trợ giữ làn cho xe thật, hướng tới Raspberry Pi 5 8GB, webcam và ESP32.
Người lái điều khiển ga, phanh và chọn hướng; hệ thống hỗ trợ góc lái.

## Hướng phát triển

- AI segmentation kết hợp OpenCV để nhận diện vạch và xác định hai biên làn.
- Raspberry Pi xử lý ảnh và tính hỗ trợ lái; ESP32 thực hiện lệnh điều khiển.
- ROS 2 kết nối các phần, hướng tới kết hợp ý định người lái và xi nhan.

Đây là hướng thiết kế cho xe thật; chưa phải các tính năng đã hoàn thiện hoặc được kiểm chứng trên phần cứng.

## Cấu trúc

| Thư mục | Nội dung |
|---|---|
| `src/` | Mã nguồn xe thật, sẽ bổ sung |
| `hardware/` | Sơ đồ đấu nối, linh kiện và cấu hình phần cứng |
| `models/` | Trọng số AI và thông tin phiên bản |
| `data/` | Dữ liệu thực tế và hướng dẫn dataset |
| `media/` | Ảnh, video thử nghiệm xe thật |
| `simulation/` | Toàn bộ mã, cấu hình, tài liệu và media mô phỏng Gazebo |

## Model và media xe thật

Đặt model tại `models/best.pt`; xem [hướng dẫn model](models/README.md).
Xem [media xe thật](media/README.md) để bổ sung ảnh/video thực tế.
Chưa kèm trọng số hoặc dataset trong repo.

## Thử nghiệm mô phỏng

Gazebo là môi trường thử nghiệm hỗ trợ quá trình phát triển. Mã và hướng dẫn nằm riêng trong [simulation/](simulation/README.md).
Bản mô phỏng hiện có ADAS geometry bằng OpenCV; không đồng nghĩa bản AI + OpenCV trên xe thật đã hoàn thiện.
Nguồn mô phỏng và giấy phép được giữ trong thư mục đó.
