#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Encoder.h> 
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ===================== WIFI & CONFIG =====================
#define UDP_LOCAL_PORT 18105    // Port nhận lệnh
#define UDP_SERVER_PORT 23151   // Port gửi Lidar
#define UDP_ODO_PORT    23152   // Port gửi Odometry

// --- CẤU HÌNH MẠNG ---
const char* pc_ip = "192.168.1.186"; 

IPAddress local_IP(192,168,1,22); // IP Robot
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

const char* ssid= "Xin Dung Xai Ke";
const char* password="0933480386NamHoang$";
// -------------------------------------------------------------

WiFiUDP UDP_Lidar; 
WiFiUDP UDP_Ctrl;   

// ===================== MPU6050 CONFIG =====================
Adafruit_MPU6050 mpu;
float gyro_z_angle = 0.0;       // Góc quay tích lũy
unsigned long last_gyro_time = 0;
float gyro_z_offset = 0.0;      // Lưu sai số tĩnh

// --- SAFETY WATCHDOG ---
unsigned long last_cmd_time = 0; 
// --------------------------------------

// ===================== UART BUFFER =====================
HardwareSerial SerialPort(2);
#define UART_BUF_SIZE 4096 
uint8_t uartBuffer[UART_BUF_SIZE];
volatile uint32_t head = 0;
volatile uint32_t tail = 0;

void pushUART(uint8_t data) {
    uartBuffer[head] = data;
    head = (head + 1) % UART_BUF_SIZE;
    if (head == tail) tail = (tail + 1) % UART_BUF_SIZE;
}

bool popUART(uint8_t &data) {
    if (head == tail) return false;
    data = uartBuffer[tail];
    tail = (tail + 1) % UART_BUF_SIZE;
    return true;
}

// ===================== LIDAR FUNCTIONS =====================
void startScan() {
    uint8_t res[7];
    uint8_t clear[1];
    unsigned long startTime;

    Serial.println("LIDAR: Resetting...");
    SerialPort.write(0xA5); SerialPort.write(0x25);
    delay(100);

    while (SerialPort.available()) SerialPort.readBytes(clear, 1);
    
    Serial.println("LIDAR: Sending Start Command...");
    SerialPort.write(0xA5); SerialPort.write(0x20);

    startTime = millis();
    while ((millis() - startTime) < 2000) { 
        if (SerialPort.available() >= 7) {
            SerialPort.readBytes(res, 7);
            if (res[0] == 0xA5 && res[1] == 0x5A) {
                Serial.println("LIDAR: OK! Connected.");
                return; 
            }
        }
        delay(10); 
    }
    Serial.println("LIDAR: Timeout! (Ignoring Lidar)");
}

void uartReadTask(void *pvParameters) {
    while (1) {
        while (SerialPort.available()) pushUART(SerialPort.read());
        vTaskDelay(1);
    }
}

void lidarTask(void *pvParameters) {
    uint8_t buf[5];
    const int BATCH = 50;
    static uint8_t packet[BATCH*5];
    static int idx = 0;

    while (1) {
        for (int i = 0; i < 5; i++) while (!popUART(buf[i])) vTaskDelay(1);

        bool S = buf[0] & 0x01;
        bool Sdao = (buf[0] >> 1) & 0x01;
        bool C = buf[1] & 0x01;

        while ((S == Sdao) || C == 0) {
            for (int i = 0; i < 4; i++) buf[i] = buf[i+1];
            while (!popUART(buf[4])) vTaskDelay(1);
            S = buf[0] & 0x01; Sdao = (buf[0] >> 1) & 0x01; C = buf[1] & 0x01;
        }

        memcpy(&packet[idx*5], buf, 5);
        idx++;
        if (idx >= BATCH) {
            UDP_Lidar.beginPacket(pc_ip, UDP_SERVER_PORT);
            UDP_Lidar.write(packet, BATCH*5);
            UDP_Lidar.endPacket();
            idx = 0;
        }
    }
}

// ===================== ENCODER =====================
#define ENC_L_A 32 
#define ENC_L_B 33 
#define ENC_R_A 18 
#define ENC_R_B 19 

ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

const float PPR = 1050.0;
const float WHEEL_DIAMETER = 0.043;
const float WHEEL_BASE = 0.178;  
const float PI_CONST = 3.1415926535;
const float METERS_PER_TICK = (PI_CONST * WHEEL_DIAMETER) / PPR;

float robot_x = 0.0;
float robot_y = 0.0;
float robot_theta = 0.0;

// ===================== MOTOR CONTROL =====================
#define IN1 15
#define IN2 2
#define ENA 27 
#define IN3 23 
#define IN4 13
#define ENB 26   

void setupMotor() {
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
    ledcSetup(0, 1000, 8); ledcAttachPin(ENA, 0);
    ledcSetup(1, 1000, 8); ledcAttachPin(ENB, 1);
}

// Task Điều khiển & Odometry
void controlOdoTask(void *pvParameters) {
    char cmd[255];
    long last_pulse_L = 0;
    long last_pulse_R = 0;
    unsigned long last_send_time = 0;

    for (;;) {
        // --- 1. ĐỌC GYRO VÀ XỬ LÝ ---
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        unsigned long current_time = millis();
        float dt = (current_time - last_gyro_time) / 1000.0;
        last_gyro_time = current_time;

        float current_z = g.gyro.z - gyro_z_offset;

        // Tích phân tính góc
        gyro_z_angle += current_z * dt; 

        // Chuẩn hóa góc về -PI đến PI
        if (gyro_z_angle > PI_CONST) gyro_z_angle -= 2 * PI_CONST;
        else if (gyro_z_angle < -PI_CONST) gyro_z_angle += 2 * PI_CONST;
        
        // --- 2. NHẬN LỆNH WIFI & WATCHDOG ---
        int packetSize = UDP_Ctrl.parsePacket();
        if (packetSize) {
            int len = UDP_Ctrl.read(cmd, 255);
            if (len > 0) {
                // >>> CẬP NHẬT THỜI GIAN NHẬN LỆNH <<<
                last_cmd_time = millis(); 
                
                char c = cmd[0];
                if (c == 'W') { // Tiến
                    digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
                    digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
                    ledcWrite(0,79); ledcWrite(1,83); 
                } 
                else if (c == 'S') { // Lùi
                    digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
                    digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
                    ledcWrite(0,80); ledcWrite(1,81);
                } 
                else if (c == 'A') { // Trái 
                    digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH);
                    digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
                    ledcWrite(0,95); ledcWrite(1,95); 
                } 
                else if (c == 'D') { // Phải
                    digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
                    digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
                    ledcWrite(0,95); ledcWrite(1,95);
                } 
                else if (c == 'X') { // Stop
                    digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
                    digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
                }
                else if (c == 'R') { // Reset
                    robot_x = 0; robot_y = 0; robot_theta = 0;
                    gyro_z_angle = 0; 
                    leftEncoder.setCount(0); 
                    rightEncoder.setCount(0);
                    last_pulse_L = 0; last_pulse_R = 0;
                }
            }
        }

        if (millis() - last_cmd_time > 300) {
            digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
            digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
        }

        // --- 3. TÍNH ODOMETRY HỢP NHẤT ---
        long curr_L = leftEncoder.getCount(); 
        long curr_R = -rightEncoder.getCount();

        long d_tick_L = curr_L - last_pulse_L;
        long d_tick_R = curr_R - last_pulse_R;

        last_pulse_L = curr_L;
        last_pulse_R = curr_R;

        if (d_tick_L != 0 || d_tick_R != 0) {
            float dist_L = d_tick_L * METERS_PER_TICK;
            float dist_R = d_tick_R * METERS_PER_TICK;
            
            float dist_center = (dist_L + dist_R) / 2.0;

            // Tính tọa độ mới
            robot_x += dist_center * cos(robot_theta);
            robot_y += dist_center * sin(robot_theta);
        }

        // Cập nhật góc liên tục
        robot_theta = gyro_z_angle;

        // --- 4. GỬI DỮ LIỆU ---
        if (millis() - last_send_time > 50) {
            UDP_Ctrl.beginPacket(pc_ip, UDP_ODO_PORT);
            UDP_Ctrl.printf("ODO:%.3f,%.3f,%.3f", robot_x, robot_y, robot_theta);
            UDP_Ctrl.endPacket();
            last_send_time = millis();
        }

        vTaskDelay(20); 
    }
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);

    // --- 1. KHỞI TẠO VÀ CÂN CHỈNH GYRO ---
    Wire.begin(); 
    if (!mpu.begin()) {
        Serial.println("LOI: Khong tim thay MPU6050!");
        while (1) { delay(10); } 
    }
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); 

    Serial.println(">>> DANG CAN CHINH GYRO... DUNG CHAM VAO XE! <<<");
    delay(1000); 

    // Lấy mẫu 200 lần để tính sai số tĩnh
    float total_z = 0;
    int samples = 200;
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        total_z += g.gyro.z;
        delay(10);
    }
    gyro_z_offset = total_z / samples; 
    
    Serial.print("Da xong! Sai so (Offset) la: ");
    Serial.println(gyro_z_offset);
    last_gyro_time = millis();
    last_cmd_time = millis(); // Khởi tạo thời gian lệnh

    // 2. Encoder
    ESP32Encoder::useInternalWeakPullResistors = puType::up; 
    leftEncoder.attachHalfQuad(ENC_L_A, ENC_L_B);
    rightEncoder.attachHalfQuad(ENC_R_A, ENC_R_B);
    leftEncoder.clearCount();
    rightEncoder.clearCount();

    // 3. WiFi
    WiFi.config(local_IP,gateway,subnet,primaryDNS,secondaryDNS);
    WiFi.begin(ssid,password);
    
    int tryCount = 0;
    while(WiFi.status()!=WL_CONNECTED && tryCount < 20){ 
        delay(500); Serial.print("."); tryCount++;
    }
    Serial.println("\nWiFi Done");
    
    UDP_Lidar.begin(UDP_SERVER_PORT); 
    UDP_Ctrl.begin(UDP_LOCAL_PORT);   

    // 4. Hardware
    setupMotor();
    SerialPort.setRxBufferSize(4096);
    SerialPort.begin(460800,SERIAL_8N1,16,17);
    startScan(); 

    // 5. Tasks
    xTaskCreatePinnedToCore(uartReadTask,"UART",2048,NULL,3,NULL,1);
    xTaskCreatePinnedToCore(lidarTask,"Lidar",4096,NULL,2,NULL,1);
    xTaskCreatePinnedToCore(controlOdoTask,"CtrlOdo",4096,NULL,1,NULL,1);
}

void loop(){ vTaskDelay(1000); }