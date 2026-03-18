 ﻿# Xe_Tu_Hanh_Lidar
 
 schematic
<img width="1358" height="829" alt="schematic" src="https://github.com/user-attachments/assets/b9e7c987-c91c-4cf7-8ae6-a2e4d605a2b2" />

VẬN HÀNH HỆ THỐNG XE TỰ HÀNH

PHẦN 1: CHUẨN BỊ VÀ CẤU HÌNH MẠNG

Bước 1: Kiểm tra IP

1.	Trên máy tính chạy ROS (Ubuntu), mở Terminal gõ ifconfig để lấy địa chỉ IP (Ví dụ: 192.168.1.3).
2.	Mở file code main.cpp của ESP32, sửa dòng: 
const char* pc_ip = "192.168.1.3";
3.	Nạp code xuống ESP32.
4.	Trên máy tính chạy App điều khiển (Windows), mở file ros_map_controller.py, sửa dòng: 	
ROBOT_IP = '192.168.1.3'

Bước 2: Khởi động Robot

1.	Bật công tắc nguồn Robot.
3.	Chờ khoảng 5-10 giây để ESP32 kết nối WiFi.
4.	Kiểm tra: Đèn LED trên ESP32 sáng/nháy báo hiệu đã kết nối (hoặc xem Serial Monitor nếu đang cắm dây).

PHẦN 2: QUY TRÌNH VẼ BẢN ĐỒ (SLAM - MAPPING)

Thực hiện bước này khi robot hoạt động ở môi trường mới chưa có bản đồ.

Bước 1: Khởi động Driver Robot (Terminal 1). Mở Terminal trên Ubuntu, chạy lệnh kết nối với phần cứng:
roslaunch my_bot run_robot.launch

Bước 2: Kích hoạt thuật toán Gmapping (Terminal 2). Mở Terminal mới, chạy lệnh bắt đầu vẽ bản đồ:
roslaunch my_bot my_gmapping.launch

Bước 3: Mở giao diện quan sát Rviz (Terminal 3)
rosrun rviz rviz

•	Add Map (Topic: /map) và LaserScan (Topic: /scan) để thấy robot đang quét.

Bước 4: Điều khiển Robot đi khám phá (Terminal 4). Sử dụng bàn phím để lái xe đi khắp phòng:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

•	Dùng các phím i (tiến), u (quay trái), o (quay phải), k (dừng), , (lùi).

•	Lưu ý: Lái chậm để bản đồ được vẽ sắc nét, không bị nhòe.
Bước 5: Lưu bản đồ (Terminal 5) Sau khi bản đồ trên Rviz đã hoàn thiện kín các bức tường, chạy lệnh:
rosrun map_server map_saver -f my_house_map

•	Kết quả: Tạo ra 2 file my_map.pgm (ảnh) và my_map.yaml (thông số) trong thư mục maps.

•	Sau bước này: Tắt tất cả các Terminal (Ctrl + C).

PHẦN 3: QUY TRÌNH DẪN ĐƯỜNG TỰ ĐỘNG (NAVIGATION)

Thực hiện bước này để chạy demo kết quả đồ án.

Bước 1: Khởi động Driver Robot (Terminal 1). Kết nối phần cứng ESP32 + Lidar:
roslaunch my_bot run_robot.launch

Bước 2: Kích hoạt hệ thống Navigation (Terminal 2). Lệnh này sẽ tải bản đồ vừa lưu, bật AMCL (định vị) và Move Base (tìm đường):
roslaunch my_bot my_navigation.launch
•	Kiểm tra: Không có dòng báo lỗi đỏ nào xuất hiện.

Bước 3: Mở cổng kết nối cho App Python (Terminal 3). Lệnh này tạo Websocket port 9090 để App Windows kết nối vào:
roslaunch rosbridge_server rosbridge_websocket.launch

PHẦN 4: SỬ DỤNG ỨNG DỤNG GIÁM SÁT & ĐIỀU KHIỂN

Bước 1: Chạy ứng dụng Mở VS Code hoặc CMD trên Windows, chạy file Python:
python ros_map_controller.py

•	Kết quả: Cửa sổ giao diện hiện lên với bản đồ màu đen trắng.
•	Trạng thái in ra Terminal: Đã kết nối thành công tới ROS!

Bước 2: Các thao tác điều khiển

1.	Quan sát vị trí:
•	Chấm đỏ: Vị trí hiện tại của Robot.
•	Mũi tên vàng: Hướng đầu xe robot đang quay.
•	Nếu chấm đỏ nhảy loạn xạ lúc đầu: Hãy lái xe một đoạn ngắn để thuật toán AMCL hội tụ vị trí chính xác.

2.	Thao tác Bản đồ:
•	Zoom (Phóng to/nhỏ): Lăn con lăn chuột.
•	Pan (Di chuyển map): Giữ chuột phải và kéo thả.

3.	Ra lệnh di chuyển (Navigation):
   
•	Chọn đích: Bấm Chuột Trái vào bất kỳ điểm trắng nào trên bản đồ.
•	Hiện tượng:
o	Một biểu tượng Hồng tâm xanh lá xuất hiện tại điểm click.
o	Robot bắt đầu tự động quay và di chuyển về phía mục tiêu.
o	Trên màn hình Rviz sẽ thấy vạch xanh lá (Global Path) vạch ra đường đi.
4.	Dừng khẩn cấp:

•	Nếu xe gặp sự cố, hãy tắt nóng App Python hoặc chạy lệnh rostopic pub /cmd_vel ... gửi vận tốc 0 về ROS. (Hoặc đơn giản nhất là nhấc bổng xe lên/tắt công tắc nguồn xe).
