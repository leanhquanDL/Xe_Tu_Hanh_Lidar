import pygame
import yaml
import roslibpy
import time
import math

# ================= CẤU HÌNH =================
ROBOT_IP = '192.168.1.186'  
MAP_YAML = 'my_map.yaml'

# ================= KẾT NỐI ROS =================
print(f"🔄 Đang kết nối tới Robot {ROBOT_IP}...")
try:
    client = roslibpy.Ros(host=ROBOT_IP, port=9090)
    client.run(timeout=5)
    print("✅ Đã kết nối thành công tới ROS!")
except Exception as e:
    print(f"❌ Lỗi kết nối: {e}")
    exit()

talker = roslibpy.Topic(client, '/move_base_simple/goal', 'geometry_msgs/PoseStamped')
listener = roslibpy.Topic(client, '/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped')

# Biến toàn cục
robot_pose = None # [x, y, theta]
current_goal = None # [x, y] - Lưu vị trí mục tiêu hiện tại

def pose_callback(msg):
    global robot_pose
    p = msg['pose']['pose']['position']
    o = msg['pose']['pose']['orientation']
    siny_cosp = 2 * (o['w'] * o['z'] + o['x'] * o['y'])
    cosy_cosp = 1 - 2 * (o['y'] * o['y'] + o['z'] * o['z'])
    theta = math.atan2(siny_cosp, cosy_cosp)
    robot_pose = [p['x'], p['y'], theta]

listener.subscribe(pose_callback)

# ================= XỬ LÝ MAP & UI =================
def load_map_config(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    return data

try:
    map_config = load_map_config(MAP_YAML)
    img_file = map_config['image']
    resolution = map_config['resolution']
    origin = map_config['origin']
    original_map_surface = pygame.image.load(img_file)
    map_w, map_h = original_map_surface.get_size()
except:
    print("❌ Lỗi load map! Kiểm tra lại file .yaml và .pgm")
    exit()

# ================= CAMERA CLASS =================
class Camera:
    def __init__(self, width, height):
        self.offset_x = 0
        self.offset_y = 0
        self.scale = 1.0
        self.drag_start = None
        self.width = width
        self.height = height

    def screen_to_world(self, sx, sy):
        x_img = (sx - self.offset_x) / self.scale
        y_img = (sy - self.offset_y) / self.scale
        y_flipped = map_h - y_img
        wx = (x_img * resolution) + origin[0]
        wy = (y_flipped * resolution) + origin[1]
        return wx, wy

    def world_to_screen(self, wx, wy):
        px = (wx - origin[0]) / resolution
        py_flipped = (wy - origin[1]) / resolution
        py = map_h - py_flipped
        sx = (px * self.scale) + self.offset_x
        sy = (py * self.scale) + self.offset_y
        return int(sx), int(sy)

    def handle_zoom(self, mouse_pos, direction):
        mx, my = mouse_pos
        old_scale = self.scale
        if direction == 1: self.scale *= 1.1
        else: self.scale /= 1.1
        self.scale = max(0.1, min(self.scale, 20.0))
        scale_change = self.scale / old_scale
        self.offset_x = mx - (mx - self.offset_x) * scale_change
        self.offset_y = my - (my - self.offset_y) * scale_change

# ================= MAIN APP =================
def send_goal(x, y):
    goal_msg = {
        'header': {'frame_id': 'map', 'stamp': {'secs': int(time.time()), 'nsecs': 0}},
        'pose': {'position': {'x': x, 'y': y, 'z': 0.0}, 'orientation': {'w': 1.0}}
    }
    talker.publish(roslibpy.Message(goal_msg))

def main():
    pygame.init()
    screen = pygame.display.set_mode((1000, 700), pygame.RESIZABLE)
    pygame.display.set_caption("Robot Controller V4.0")
    clock = pygame.time.Clock()
    cam = Camera(1000, 700)
    
    # Căn giữa map
    cam.offset_x = (1000 - map_w) / 2
    cam.offset_y = (700 - map_h) / 2
    
    global current_goal
    font = pygame.font.SysFont("Arial", 12)
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.VIDEORESIZE: cam.width, cam.height = event.w, event.h
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # CLICK CHUỘT TRÁI
                    wx, wy = cam.screen_to_world(event.pos[0], event.pos[1])
                    current_goal = [wx, wy] # Lưu tọa độ THỰC TẾ
                    send_goal(wx, wy)
                    
                elif event.button == 3: cam.drag_start = event.pos
                elif event.button == 4: cam.handle_zoom(event.pos, 1)
                elif event.button == 5: cam.handle_zoom(event.pos, -1)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3: cam.drag_start = None
            
            elif event.type == pygame.MOUSEMOTION:
                if cam.drag_start:
                    dx, dy = event.pos[0] - cam.drag_start[0], event.pos[1] - cam.drag_start[1]
                    cam.offset_x += dx
                    cam.offset_y += dy
                    cam.drag_start = event.pos

        # --- VẼ ---
        screen.fill((30, 30, 30))

        # 1. Map
        scaled_w, scaled_h = int(map_w * cam.scale), int(map_h * cam.scale)
        if scaled_w > 0 and scaled_h > 0:
            scaled_map = pygame.transform.scale(original_map_surface, (scaled_w, scaled_h))
            screen.blit(scaled_map, (cam.offset_x, cam.offset_y))

        # 2. VẼ MỤC TIÊU (TARGET)
        if current_goal:
            # Chuyển từ tọa độ thực -> tọa độ màn hình hiện tại
            gx, gy = cam.world_to_screen(current_goal[0], current_goal[1])
            
            # Vẽ tâm (Crosshair)
            color = (0, 255, 0) # Xanh lá
            size = 10
            pygame.draw.circle(screen, color, (gx, gy), 4) 
            pygame.draw.circle(screen, color, (gx, gy), 10, 1) 
            pygame.draw.line(screen, color, (gx - size, gy), (gx + size, gy), 1) 
            pygame.draw.line(screen, color, (gx, gy - size), (gx, gy + size), 1) 
            
            # Text tọa độ
            txt = font.render(f"Goal", True, color)
            screen.blit(txt, (gx + 5, gy - 20))

        # 3. VẼ ROBOT (NHỎ GỌN)
        if robot_pose:
            rx, ry = cam.world_to_screen(robot_pose[0], robot_pose[1])
            theta = robot_pose[2]
            
            robot_radius = max(3, int(4 * cam.scale)) 
            
            # Thân xe (Màu đỏ)
            pygame.draw.circle(screen, (255, 50, 50), (rx, ry), robot_radius)
            
            head_len = robot_radius * 2.5
            hx = rx + head_len * math.cos(theta)
            hy = ry - head_len * math.sin(theta)
            pygame.draw.line(screen, (255, 255, 0), (rx, ry), (hx, hy), max(1, int(cam.scale)))

        pygame.display.flip()
        clock.tick(60)

    client.terminate()
    pygame.quit()

if __name__ == '__main__':
    main()