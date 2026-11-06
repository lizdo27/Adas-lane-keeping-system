# Dữ liệu xe thật — bản pilot

Dataset và ảnh chưa được phát hành trong repository. ZIP dùng để train là `ADAS_REAL_MERGED.zip`; giữ bản gốc trong kho riêng của nhóm. Một số nguồn có điều kiện sử dụng riêng, cần kiểm tra quyền trước khi công khai.

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
