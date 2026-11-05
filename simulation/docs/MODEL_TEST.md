# Thử model trong Gazebo

Model dùng chung đặt tại `models/best.pt` từ gốc repo.



Model cần là YOLO segmentation, có class `lane_marking`; `crosswalk` dùng để loại vùng vạch qua đường. Model lane_area cũ không thay thế trực tiếp được.

Từ thư mục `simulation/`, sau khi cài Ultralytics/PyTorch tương thích trong môi trường Python:

```bash
source ./bashr
basic_autonomy/.venv/bin/python basic_autonomy/adas_view.py --perception hybrid --weights ../models/best.pt --speed 0.06
```

Phải truyền `--weights`: chương trình chưa mặc định tìm ở models/ gốc repo. Đây là chế độ hybrid thử nghiệm, cần kiểm tra độ trễ và nhận diện trước khi dùng; không phải thuật toán kết hợp mới đã được kiểm chứng.

