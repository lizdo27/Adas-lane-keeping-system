# ADAS — BFMC Gazebo / ROS 2

Bản gọn của mô phỏng đang dùng `map_with_car_light.launch`, kèm ADAS hỗ trợ giữ làn.
Người lái giữ ga, phanh và chọn hướng. ADAS hỗ trợ góc lái bằng xử lý ảnh hình học OpenCV.
Bản chạy mặc định không cần model AI. Các module AI được giữ vì entrypoint có import tùy chọn; chưa kèm trọng số để chạy các chế độ đó.

## Nội dung

- `src/sim_pkg`: launch chính, Gazebo, bridge, spawn xe và world đang sử dụng.
- `src/models_pkg`: 9 mô hình được tham chiếu, gồm track, separator và các bộ phận xe.
- `src/plugins_pkgs`, `src/utils`: plugin và thư viện phụ thuộc của launch.
- `basic_autonomy`: nhận diện, ADAS, giao diện, ghi clip và cấu hình camera.
- Không kèm trọng số, dataset, clip, môi trường Python, build/install/log hoặc lịch sử Git gốc.

Giữ cấu trúc package ROS để build; không đổi tên tùy tiện các package.

## Cài đặt và chạy

Môi trường gốc: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.
Xem [hướng dẫn cài mô phỏng gốc](docs/UPSTREAM_SETUP.md) để cài các phụ thuộc ROS/Gazebo và thư viện phát triển plugin. Tài liệu gốc còn mô tả tính năng không thuộc bản gọn này.

Các gói bổ sung cho giao diện và xử lý ảnh:

```bash
sudo apt install python3-opencv python3-numpy python3-pil python3-venv fonts-dejavu-core ros-jazzy-cv-bridge ros-jazzy-ros-gz
```

Từ thư mục gốc repo, chuyển vào `simulation/` rồi build sau khi cài đầy đủ phụ thuộc:

```bash
cd simulation
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 -m venv --system-site-packages basic_autonomy/.venv
ros2 launch sim_pkg map_with_car_light.launch
```

Terminal thứ hai, bắt đầu tại thư mục gốc repo:

```bash
cd simulation
source ./bashr
basic_autonomy/.venv/bin/python basic_autonomy/adas_view.py --perception geometry --speed 0.06
```

W/A/S/D lái; Space phanh; H bật/tắt hỗ trợ; R ghi clip; T kết thúc clip; Q thoát.
0.06 là giá trị lệnh điều khiển, không phải vận tốc đo bằng m/s.
Giao diện mặc định 90% bản gốc; thêm `--display-scale 1` để khôi phục.
Cấu hình hình học dành cho camera Gazebo hiện tại; phải hiệu chuẩn lại cho xe thật.

## Kiểm chứng

Kiểm tra tích hợp ngày 05/11/2026: cú pháp 17 file Python và cấu trúc 22 file XML/launch/model hợp lệ; 130 file trong manifest khớp SHA256. Build ROS 2 Jazzy bằng `colcon build` thành công cả 8 package.
Chưa chạy Gazebo hoặc kiểm thử giao diện tương tác từ bản tích hợp; build thành công không thay thế thử chạy hoàn chỉnh.

## Nguồn gốc

Mô phỏng kế thừa [UniPD-DriveOps-BFMC/SimulatorROS2](https://github.com/UniPD-DriveOps-BFMC/SimulatorROS2).
Giữ LICENSE, copyright và thông tin package gốc. Phần bổ sung chính nằm trong `basic_autonomy`.
`EXPORT_MANIFEST.json` ghi danh sách file sao chép và SHA256 để đối chiếu.

Bản mô phỏng được tích hợp trong `simulation/` của repo ADAS; chạy các lệnh ROS 2 và ADAS từ thư mục này. Manifest ghi hash tại thời điểm xuất, trước khi điều chỉnh README cho vị trí tích hợp.
