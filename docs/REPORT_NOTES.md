# Nội dung dùng cho báo cáo

## Mô tả phương pháp

Đề tài xây dựng nguyên mẫu nhận diện vạch đường phục vụ hệ thống ADAS hỗ trợ giữ làn trên xe mô hình. Phương pháp kết hợp mạng YOLOv8n-seg với xử lý ảnh OpenCV. Mạng phân đoạn cung cấp thông tin ngữ nghĩa về vạch đường và vạch qua đường; xử lý ảnh khai thác độ sáng, tương phản hai phía và hình dạng của dải sơn. Các nguồn bằng chứng được hợp nhất có điều kiện thay vì cộng toàn bộ mask. Bằng chứng từ các frame trước được bù chuyển động, giới hạn thời gian tồn tại và đối chiếu với ảnh hiện tại để giảm nhấp nháy và vùng phản chiếu bị nhận nhầm.

## Kết quả và giới hạn

Nhóm đã xây dựng quy trình chuẩn bị dữ liệu pilot, huấn luyện model hai lớp và chương trình thử nghiệm trên video thực tế. Phiên bản hybrid bổ sung cơ chế kiểm tra điểm tựa hướng vạch và chuỗi vạch đứt. Kết quả quan sát tại một số đoạn cho thấy giảm phản chiếu và bổ sung một số đoạn AI bỏ sót; chưa đủ bằng chứng để kết luận cải thiện độ chính xác trên toàn bộ dữ liệu. Mô hình còn hạn chế với ánh sáng phản chiếu, hình học giao lộ và nhãn chưa thống nhất. Các chỉ số validation, điều kiện đo và giới hạn tập dữ liệu được trình bày riêng trong tài liệu thử nghiệm.

## Phạm vi đã hoàn thành và hướng tiếp theo

Code xe thật hiện dừng ở tầng nhận diện offline. Bộ hỗ trợ góc lái chạy trong Gazebo là nhánh mô phỏng riêng sử dụng xử lý ảnh hình học. Triển khai ROS 2 trên Raspberry Pi 5, chọn cặp biên làn, tính sai lệch và giao tiếp bộ chấp hành xe thật là các bước tiếp theo cần kiểm thử. Báo cáo không đồng nhất kết quả mô phỏng với kết quả vận hành xe thật.

## Nhóm bổ sung trước khi nộp

- Ảnh xe, danh sách linh kiện, sơ đồ đấu dây và thông số camera thực tế.
- Video xe thật, điều kiện ghi hình và mô tả thao tác của người lái.
- Ground truth và phép đo bổ sung nếu có; không điền số liệu dự kiến thành kết quả đã đo.
- Phiên bản model, môi trường chạy, SHA256 trọng số và quyền sử dụng dữ liệu.

Nguồn chi tiết: [thuật toán](ALGORITHM.md), [kiến trúc](ARCHITECTURE.md), [thử nghiệm](EXPERIMENTS.md), [dữ liệu](../data/README.md).
