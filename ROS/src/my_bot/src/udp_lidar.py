#!/usr/bin/env python3
import rospy
import socket
import struct
import math
from sensor_msgs.msg import LaserScan

# CẤU HÌNH MẠNG
UDP_IP = "0.0.0.0"
UDP_PORT = 23151  # Port Lidar

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def main():
    rospy.init_node('udp_lidar_node')
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
    
    print("Lidar Driver Started! Waiting for UDP data on port 23151...")

    # Cấu hình bản tin LaserScan
    scan = LaserScan()
    scan.header.frame_id = "laser_frame" # Tên khung hình của Lidar
    scan.angle_min = 0.0
    scan.angle_max = 2.0 * math.pi
    scan.angle_increment = (2.0 * math.pi) / 360.0 
    scan.time_increment = 0.0
    scan.range_min = 0.12 # 12cm
    scan.range_max = 8.0  # 8m
    
    # Mảng chứa dữ liệu khoảng cách (360 độ)
    ranges = [float('inf')] * 360
    
    last_pub_time = rospy.get_time()

    while not rospy.is_shutdown():
        try:
            # Nhận dữ liệu thô từ ESP32
            data, addr = sock.recvfrom(1024)
            
            # ESP32 gửi gói tin gồm nhiều cụm 5 bytes
            # Mỗi cụm 5 bytes là 1 điểm đo: [Chất lượng, Góc L, Góc H, KC L, KC H]
            num_points = len(data) // 5
            
            for i in range(num_points):
                base = i * 5
                # Đọc 5 bytes
                b0 = data[base]
                b1 = data[base+1]
                b2 = data[base+2]
                b3 = data[base+3]
                b4 = data[base+4]

                # Giải mã theo chuẩn RPLIDAR
                angle_deg = ((b1 >> 1) | (b2 << 7)) / 64.0
                
                # Khoảng cách
                dist_mm = (b3 | (b4 << 8)) / 4.0
                dist_m = dist_mm / 1000.0

                # Lọc góc hợp lệ
                if 0 <= angle_deg < 360:
                    index = int(angle_deg)
                    
                    if dist_m > scan.range_min and dist_m < scan.range_max:
                        ranges[index] = dist_m
                    else:
                        ranges[index] = float('inf')

            # Publish bản tin mỗi 0.1 giây (10Hz) để Rviz đỡ giật
            if rospy.get_time() - last_pub_time > 0.1:
                scan.header.stamp = rospy.Time.now()
                scan.ranges = ranges
                scan_pub.publish(scan)
                # Reset mảng sau khi gửi
                # ranges = [float('inf')] * 360 
                last_pub_time = rospy.get_time()

        except Exception as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    main()
