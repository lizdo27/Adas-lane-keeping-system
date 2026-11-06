# ADAS Lane Keeping System

Dự án ADAS cho **xe thật**, hướng tới Raspberry Pi 5 8GB, camera và ESP32. Người lái quyết định ga, phanh và hướng di chuyển; hệ thống hướng tới hỗ trợ giữ làn. ROS 2 là kiến trúc tích hợp dự kiến, OpenCV và YOLO là các thành phần nhận diện.

Hiện đã có model segmentation pilot, code nhận diện AI/OpenCV trên video thật và một nhánh mô phỏng Gazebo riêng. **Chưa hoàn thành bộ điều khiển hybrid trên xe thật; chất lượng AI vẫn cần cải thiện.**

## Tài liệu báo cáo

- [Thuật toán hợp nhất AI + OpenCV v3](docs/ALGORITHM.md): luồng xử lý, điều kiện chọn mask, theo dõi và hạn chế.
- [Kiến trúc và trạng thái từng thành phần](docs/ARCHITECTURE.md).
- [Kết quả thử nghiệm](docs/EXPERIMENTS.md): validation, video v2 đầy đủ và kiểm tra mẫu v3.
- [Nội dung mô tả dùng cho báo cáo](docs/REPORT_NOTES.md).
- [Dữ liệu và cách chia tập](data/README.md).

## Chạy nhận diện video thật

Model segmentation hai lớp `lane_marking`, `crosswalk` đã có tại `models/best.pt` qua Git LFS. Chạy `git lfs install` và `git lfs pull` sau khi clone để tải trọng số. Dùng video gốc chưa có overlay:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python src/perception/ai_video.py --video /duong/dan/video_goc.mp4
python src/perception/hybrid_video.py --video /duong/dan/video_goc.mp4
```

Có thể truyền `--weights /duong/dan/best_real.pt` và `--no-show`. Các chương trình xử lý toàn bộ video và ghi kết quả cạnh video nguồn. Q chỉ ẩn cửa sổ; Ctrl+C mới dừng. Hybrid xuất mask vạch nhìn thấy, chưa xuất góc lái hay cặp biên làn đã xác nhận.

[Hướng dẫn đầy đủ](src/perception/README.md) · [Notebook Colab](training/Train_ADAS_REAL.ipynb) · [Hướng dẫn train](training/README.md) · [Nơi đặt model](models/README.md)

## Cấu trúc

| Thư mục | Nội dung |
|---|---|
| `src/perception/` | Code AI-only và hybrid trên video thật |
| `tests/` | Kiểm tra logic hợp nhất mask |
| `training/` | Notebook train pilot trên Colab |
| `docs/` | Thuật toán, kiến trúc, kết quả và nội dung báo cáo |
| `models/` | Trọng số pilot best.pt qua Git LFS, metadata và hướng dẫn |
| `data/` | ZIP dataset pilot qua Git LFS và báo cáo hợp nhất |
| `hardware/` | Nhóm bổ sung linh kiện và sơ đồ xe thật |
| `media/` | Nhóm bổ sung ảnh/video xe thật |
| `simulation/` | Toàn bộ code, tài liệu và media Gazebo |

Kiểm tra logic không cần trọng số:

```bash
python tests/test_fusion.py
```

## Mô phỏng

[Bản Gazebo / ROS 2](simulation/README.md) sử dụng `map_with_car_light.launch` và ADAS hình học OpenCV. Tham số mô phỏng không được mặc định dùng cho xe thật. Kết quả chạy trong Gazebo không chứng minh chất lượng AI hoặc độ an toàn của xe thật.

## Giấy phép và tài nguyên ngoài

Mã của dự án theo [LICENSE](LICENSE). Thành phần mô phỏng giữ [giấy phép riêng](simulation/LICENSE). Thư viện Ultralytics có điều kiện cấp phép riêng; giấy phép repo không thay thế giấy phép thư viện, model hoặc dữ liệu nguồn. Trọng số pilot đã phát hành qua Git LFS. ZIP dataset pilot cũng đã phát hành qua Git LFS; thông tin phần cứng do nhóm bổ sung. Xem [danh mục bàn giao](docs/DELIVERY.md) để biết phần đã có và phần nhóm cần bổ sung.
