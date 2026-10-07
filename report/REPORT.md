# Báo cáo Day 6: LiDAR-Camera Projection QA & Calibration Drift Sensitivity

> Thay **mọi** ô có chữ ĐIỀN nằm trong ngoặc vuông bằng nội dung của bạn, xoá luôn cả dấu ngoặc vuông. Lệnh `python tools/check_submission.py` sẽ báo FAIL nếu còn sót bất kỳ chỗ nào.

- **Họ tên:** Vũ Gia Khải
- **MSSV:** 2A202602786
- **Lớp:** K4-Track4
- **Link repo:** https://github.com/vukhai248/K4-Track4-Day06-VuGiaKhai-2A202602786-3D-From-Point-Clouds
- **Topic:** A — LiDAR-camera projection QA
- **Dataset:** data/synthetic, data/kitti_mini, data/nuscenes_mini_subset
- **Các frame đã dùng:** 000000, 000011, 000021, 000049, scene-0103_010

> Hãy viết ngắn: mỗi mục từ 3 đến 8 dòng, ưu tiên số liệu và hình ảnh.

## 1. Claim

Độ lệch góc xoay Yaw từ 1.0° trở lên hoặc sai số dịch chuyển t_y từ 0.1 m giữa LiDAR và Camera làm giảm hơn 30% tỷ lệ điểm LiDAR rơi đúng vào bounding box 2D của vật thể (Point-in-Box ratio), đồng thời sự suy giảm này diễn ra nhanh gấp đôi đối với các vật thể ở khoảng cách xa (>30 m) so với vật thể ở cự ly gần (<15 m), có thể được phát hiện tự động bằng bộ giám sát consistency giữa 3D box projection và 2D detection box.

## 2. Evidence

Bảng hoặc plot số liệu, kèm ảnh/video demo. Ghi rõ đường dẫn file trong `results/`.

| Cấu hình / mức perturb | Metric 1 | Metric 2 | Ghi chú |
|---|---|---|---|
| [ĐIỀN] | | | |

![demo](../results/figures/[ĐIỀN].png)

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Các lệnh tái tạo lại toàn bộ kết quả từ repo sạch.

```bash
[ĐIỀN]
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| [ĐIỀN] | | |
