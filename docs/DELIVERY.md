# Danh mục bàn giao GitHub

Cập nhật 06/11/2026. Dự án chính là xe thật; toàn bộ tài nguyên Gazebo giữ trong `simulation/`.

| Hạng mục | Nơi lưu | Trạng thái |
|---|---|---|
| Model pilot hai lớp | models/best.pt | Đã phát hành bằng Git LFS |
| Danh tính model và SHA256 | models/model_metadata.json | Đã xác minh từ file thực tế |
| AI-only, hybrid v3 | src/perception/ | Có code chạy video, chưa điều khiển xe |
| Kiểm tra logic fusion | tests/test_fusion.py | Chạy đạt |
| Notebook train | training/ | Có notebook và hướng dẫn Colab |
| Thống kê dataset | data/merge_report.json | Có split, số đối tượng, giới hạn và SHA256 ZIP |
| Thuật toán và báo cáo | docs/ | Có mô tả phương pháp, kiến trúc, số liệu và giới hạn |
| Mô phỏng, ảnh/video Gazebo | simulation/ | Đã có; tách khỏi dữ liệu xe thật |
| Linh kiện và sơ đồ xe thật | hardware/ | Nhóm tự bổ sung theo phần cứng thực tế |
| Ảnh/video xe thật để báo cáo | media/ | Nhóm chọn và bổ sung |
| Dataset huấn luyện | data/ADAS_REAL_MERGED.zip | ZIP pilot nguyên bản, phát hành qua Git LFS |
| ROS 2 và firmware xe thật | Chưa triển khai | Không có code để công bố là đã hoàn tất |

Không đưa lên repo các môi trường .venv, thư mục build/install/log, cache, dataset ZIP riêng, token hoặc output thử nghiệm dung lượng lớn. Code nhận diện giữ khả năng tạo lại video so sánh từ video gốc và model.

Đối với báo cáo, dùng ALGORITHM.md cho phương pháp, EXPERIMENTS.md cho kết quả đã có và REPORT_NOTES.md cho phần mô tả. Không dùng kết quả kiểm tra suy luận trên ảnh tổng hợp thay cho đánh giá chất lượng ngoài đời.
