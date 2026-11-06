# Dữ liệu xe thật — bản pilot

ZIP đã dùng train: **[ADAS_REAL_MERGED.zip](ADAS_REAL_MERGED.zip)**, phát hành bằng Git LFS theo yêu cầu của nhóm. File giữ nguyên byte so với bản pilot đã hợp nhất, dung lượng 57.131.033 byte (khoảng 54,5 MiB).

Tải repo bằng Git rồi chạy:

```bash
git lfs install
git lfs pull --include="data/ADAS_REAL_MERGED.zip"
sha256sum data/ADAS_REAL_MERGED.zip
```

SHA256 phải khớp `dataset_zip_sha256` trong [merge_report.json](merge_report.json). Nếu file chỉ vài trăm byte thì đó là pointer LFS, chưa phải ZIP thật. Upload ZIP tải được vào gốc My Drive để chạy [notebook Colab](../training/Train_ADAS_REAL.ipynb).

ZIP gồm ảnh, polygon nhãn, data.yaml, manifest.json và báo cáo hợp nhất. Giữ nguyên các lưu ý nguồn dữ liệu trong ZIP; giấy phép mã nguồn của repo không tự cấp lại giấy phép cho dữ liệu.

Hai class sau hợp nhất: `0: lane_marking`, `1: crosswalk`. Vạch trái/phải dùng chung lớp; chọn hai biên phù hợp là nhiệm vụ hậu xử lý, không mặc định chỉ có hai instance trong ảnh.

## Thống kê hợp nhất

6 ZIP nguồn đã có augmentation, tổng 3.361 ảnh. Bản pilot giữ 1.806 ảnh:

| Split | Ảnh | lane_marking | crosswalk |
|---|---:|---:|---:|
| train | 1733 | 6355 | 2384 |
| valid | 41 | 97 | 53 |
| test | 32 | 155 | 4 |

Đã loại 749 ảnh có nhãn polygon không hợp lệ hoặc nhãn trống chưa xác minh, 14 bản trùng pixel, 236 ảnh ở vùng đệm ranh giới block và 556 biến thể augmentation không dùng cho đánh giá. Việc loại cả ảnh có một nhãn sai có thể bỏ mất dữ liệu tốt; nhãn trống cũng có thể là negative hợp lệ nên cần duyệt lại trước phiên bản sau.

Chia theo block 100 frame, vùng đệm 5 frame mỗi đầu/cuối, giữ cùng family ảnh/biến thể và hash trong một split. Đây vẫn chủ yếu là cùng chuỗi quay; không đảm bảo độc lập theo buổi quay hoặc cảnh. Không chia ngẫu nhiên lại các ảnh augmentation.

Crosswalk còn lẫn polygon từng thanh với polygon cả vùng. Nhóm chấp nhận cho pilot nhận biết hiện diện, nhưng inconsistency này vẫn ảnh hưởng train và đánh giá. Có 11 trường hợp ảnh trùng nhưng nhãn khác trong nhóm duplicate; bản pilot giữ nhãn xuất hiện trước, chưa giải quyết khác biệt bằng duyệt tay. Không xem tập đã “làm sạch hoàn toàn”.

Phiên bản tiếp theo nên thống nhất quy tắc crosswalk, duyệt nhãn và tăng ảnh test từ buổi quay độc lập. Để tái lập cần giữ ZIP, `manifest.json`, `merge_report.json`, danh sách loại và SHA256 cùng bản model.

Báo cáo máy đọc được và SHA256 của ZIP pilot: [merge_report.json](merge_report.json). Ảnh và manifest nằm trong ZIP; không giải nén chúng vào lịch sử Git.
