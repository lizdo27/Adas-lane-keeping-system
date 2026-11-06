# Kết quả thử nghiệm và giới hạn bằng chứng

Tài liệu tổng hợp số liệu pilot đã có, không phải chứng nhận hoàn thiện ADAS. Cập nhật 06/11/2026.

## Huấn luyện và validation

YOLOv8n-seg, ảnh đầu vào 640, batch 8, seed 42, tối đa 100 epoch, patience 20. Lượt train `real_v8n_seg_20261105_091718` dừng sớm ở epoch 41; checkpoint tốt nhất tại epoch 21. Số liệu dưới đây được chép từ kết quả validation người thực hiện cung cấp, chưa chạy lại để tái lập.

| Lớp | Số instance validation | Mask precision | Mask recall | Mask mAP50 | Mask mAP50–95 |
|---|---:|---:|---:|---:|---:|
| lane_marking | 97 | 0.778 | 0.557 | 0.601 | 0.300 |
| crosswalk | 53 | 0.508 | 0.566 | 0.445 | 0.157 |
| Tổng | 150 | 0.643 | 0.561 | 0.523 | 0.228 |

Validation có 41 ảnh; crosswalk xuất hiện trong 23 ảnh. Đây là metrics segmentation validation, không phải độ chính xác điều khiển hoặc kết quả trên tập test độc lập. Chưa có số liệu test đã được xác nhận để công bố.

## Chạy đầy đủ video với hybrid v2

Nguồn `final_video_20261102_172131.mp4`: 3.221 frame, 30 FPS, khoảng 107,37 giây. Run `hybrid_masks_20261105_184132` đã ghi đủ frame. Log ghi trung bình AI khoảng 167,46 ms và CV/fusion khoảng 130,93 ms, bỏ frame đầu khỏi trung bình.

Tổng hai khâu khoảng 298,39 ms/frame; chưa gồm toàn bộ đọc/hiển thị/ghi video. Đây là lần chạy trên máy phát triển, **không phải benchmark Pi 5**, cũng không phải số đo của v3. Có 275 frame mask rỗng; không được kết luận đó là 275 lỗi khi chưa có ground truth.

## Sửa v3 và kiểm tra đoạn ngắn

V2 còn nhận nhầm phản chiếu sáng tồn tại lâu thành vạch. V3 thêm kiểm tra hướng/vị trí với điểm tựa AI mới, giới hạn tuổi điểm tựa và bằng chứng chuỗi đoạn đứt độc lập. Đã so sánh 9 đoạn ngắn, mỗi đoạn 8 frame, quanh các chỉ số 400, 1000, 1300, 1600, 1900, 2200, 2500, 2800, 3100.

| Mẫu frame | Pixel CV thêm ở v2 | Pixel CV thêm ở v3 | Nhận xét khi xem mẫu |
|---|---:|---:|---|
| 1300 | 1801 | 25 | Giảm vùng phản chiếu thêm nhầm |
| 1600 | 44 | 929 | Bổ sung được đoạn vạch trái bị AI bỏ sót |
| 2200 | 3391 | 176 | Giảm vùng phản chiếu thêm nhầm |

Các giá trị là đếm pixel, **không phải điểm accuracy**. Quan sát mẫu chỉ cho thấy thay đổi có ích tại các đoạn đó; chưa chứng minh v3 tốt hơn trên toàn bộ video. Chưa xác nhận một lượt chạy đầy đủ v3 trong tài liệu này.

Test tổng hợp kiểm tra: không chấp nhận một dải sáng đơn độc chỉ vì tồn tại lâu; phục hồi chuỗi đoạn đứt; không lấp khoảng đứt; không lưu bóng mask khi frame mới trống; giữ AI khi CV không có; bỏ lịch sử khi timestamp đảo chiều. Những test này kiểm tra logic, không đo chất lượng nhận diện thực tế.

## Đánh giá cần bổ sung

1. Video từ lần quay khác để tách khỏi cảnh huấn luyện; bao gồm thẳng, cua, ngã tư, mờ và phản chiếu.
2. Ground truth thống nhất; chạy AI-only/CV/hybrid trên cùng frame và cùng ROI.
3. Mask IoU/precision/recall từng lớp; sai số biên làn và tỷ lệ mất cặp biên sau khi có bộ chọn làn.
4. Độ ổn định theo thời gian, độ trễ p50/p95, bộ nhớ và nhiệt độ trên Pi 5.
5. Kiểm thử điều khiển riêng sau khi perception đủ tin cậy, có người lái chủ động ngắt hỗ trợ.

Không điều chỉnh tham số theo tập test rồi dùng lại tập đó để báo cáo kết quả cuối.
