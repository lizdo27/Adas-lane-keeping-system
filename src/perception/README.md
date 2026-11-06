# Thử nhận diện trên video xe thật

Hai chương trình chạy offline, không gửi lệnh lái/ga/phanh và không cần ROS 2.

Từ thư mục gốc repository:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python src/perception/ai_video.py --video /duong/dan/video_goc.mp4
python src/perception/hybrid_video.py --video /duong/dan/video_goc.mp4
```

Chép model segmentation vào `models/best.pt`, hoặc truyền `--weights /duong/dan/best_real.pt`.
Dùng video **gốc**, không dùng video đã có overlay AI làm đầu vào hybrid vì chữ và màu tô gây nhiễu.

| Tham số | Mặc định / ý nghĩa |
|---|---|
| `--video` | Bắt buộc; video nguồn |
| `--weights` | `models/best.pt` tính từ gốc repo |
| `--device` | `cpu`; dùng `0` nếu có CUDA tương thích |
| `--conf` | 0.25, ngưỡng lấy dự đoán từ YOLO |
| `--roi-top` | Hybrid: 0.40, bỏ phần trên 40% ảnh |
| `--no-show` | Không mở cửa sổ, vẫn lưu toàn bộ video |

Q hoặc đóng cửa sổ chỉ ẩn xem trước; Ctrl+C dừng và kết quả chỉ còn một phần.
Output lưu **cạnh video nguồn**. AI tạo video có hậu tố `_ai_test_...mp4`.
Hybrid tạo thư mục `hybrid_masks_v3_<timestamp>/` gồm:

- `hybrid_full.mp4`: mask hợp nhất trên ảnh nguồn.
- `comparison_full.mp4`: ảnh gốc, AI, ứng viên OpenCV và hybrid.
- `frames.jsonl`: thời gian xử lý và lượng bằng chứng ở từng frame.
- `summary.json`: số frame, hoàn tất hay chưa, tham số và thời gian trung bình.

Video được ghi theo FPS nguồn; tốc độ phát lại không phải tốc độ tính toán.
Trong bảng hybrid, AI và CV dùng cùng ROI. Crosswalk dùng nội bộ để hạn chế bổ sung nhầm vạch ngang; bảng AI chỉ hiện lane_marking.

Kiểm tra logic mask (không cần model):

```bash
python tests/test_fusion.py
```

Thuật toán và giới hạn: [ALGORITHM](../../docs/ALGORITHM.md).
Các ngưỡng pixel hiện phụ thuộc độ phân giải, góc camera và ánh sáng đã thử; chưa tự hiệu chuẩn cho webcam khác.
