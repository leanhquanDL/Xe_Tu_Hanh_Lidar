#!/usr/bin/env python3
import rospy
import socket
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math

# ================= CẤU HÌNH MẠNG =================
UDP_IP = "0.0.0.0"
UDP_PORT = 23152

# Port để gửi lệnh điều khiển xuống ESP32
ESP_IP = "192.168.1.22" # <--- IP ROBOT 
ESP_PORT = 18105

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.1)

def main():
    rospy.init_node('esp32_driver')
    
    # Publisher cho Odometry
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    # Subscriber cho Cmd_vel (Bàn phím)
    def cmd_vel_callback(msg):
        cmd = ""
        if msg.linear.x > 0: cmd = "W"
        elif msg.linear.x < 0: cmd = "S"
        elif msg.angular.z > 0: cmd = "A" # Quay Trái
        elif msg.angular.z < 0: cmd = "D" # Quay Phải
        else: cmd = "X"
        
        # Gửi lệnh xuống ESP32
        sock.sendto(cmd.encode(), (ESP_IP, ESP_PORT))

    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

    rate = rospy.Rate(20) # 20Hz

    x = 0.0
    y = 0.0
    th = 0.0

    print("Driver Started! Waiting for UDP data from ESP32...")

    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(1024) 
            line = data.decode('utf-8').strip()
            
            print(f"DATA DEN: {line}")

            # Kiểm tra xem có phải gói tin ODO không
            if line.startswith("ODO:"):
                # Dữ liệu dạng: ODO:x,y,theta
                parts = line.split(':')[1].split(',')
                if len(parts) == 3:
                    x = float(parts[0])
                    y = float(parts[1])
                    th = float(parts[2]) 

                    # --- CHUYỂN ĐỔI GÓC SANG QUATERNION ---
                    # Rviz chỉ hiểu Quaternion, không hiểu góc thường
                    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

                    # 1. Gửi TF (Transform) - Để Rviz biết khung xe nằm đâu
                    odom_broadcaster.sendTransform(
                        (x, y, 0.),
                        odom_quat,
                        rospy.Time.now(),
                        "base_link",
                        "odom"
                    )

                    # 2. Gửi Topic /odom 
                    odom = Odometry()
                    odom.header.stamp = rospy.Time.now()
                    odom.header.frame_id = "odom"

                    # Vị trí
                    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

                    # Vận tốc
                    odom.child_frame_id = "base_link"
                    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

                    odom_pub.publish(odom)
                    
                    # Debug in ra màn hình để bạn yên tâm
                    # print(f"Received: X={x:.2f} Y={y:.2f} Theta={th:.2f}")

        except socket.timeout:
            pass # Không có dữ liệu thì bỏ qua
        except Exception as e:
            print(f"Error: {e}")

        rate.sleep()

if __name__ == '__main__':
    main()
