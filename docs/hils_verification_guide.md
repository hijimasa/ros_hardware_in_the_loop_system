# HILS 動作確認手順書

各エミュレータの動作確認手順をまとめる。検証は「1台PC内ループバック」→「2台PC間」の順で進める。

---

## 1. LiDARエミュレータ（2台PC or 1台PC内ループバック）

LiDARエミュレータはマイコン不要。USB-LANアダプタまたは1台PC内のループバックで検証できる。

### 1.1 Livox Mid-360（実装・動作確認済み）

省略（既存の検証手順に従う）。

### 1.2 Velodyne VLP-16

#### 1台PC内検証（Docker Compose で 2 コンテナ）

`dummy0` 方式ではエミュレータとドライバが同じカーネルのポート2368を取り合って衝突する。Dockerコンテナを2つ立ち上げれば、各コンテナが独立したネットワーク名前空間を持つためポート衝突なく検証できる。

```bash
# Docker Composeで sim/robot 両コンテナを起動
cd docker
docker compose up -d

# コンテナ 'sim' 側でエミュレータを起動
docker compose exec sim bash
# コンテナ内:
source /opt/ros/jazzy/setup.bash
cd ~/colcon_ws && colcon build && source install/setup.bash
ros2 launch hils_bridge_lidar_velodyne velodyne_emulator.launch.py \
  device_ip:=192.168.100.201 \
  host_ip:=192.168.100.100 \
  network_interface:=eth0
  # 点群のトピックを指定したい場合は pointcloud_topic:=/livox/lidar
# 別ターミナルでテスト用PointCloud2をパブリッシュ
# ros2 bag play などでもOK

# 別ターミナルでコンテナ 'robot' 側でドライバを起動
docker compose exec robot bash
# コンテナ内:
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
# launchファイルではYAMLファイルの変更が必要なのでrunからの立ち上げが手軽
ros2 run velodyne_driver velodyne_driver_node --ros-args \
  -p device_ip:=192.168.100.201 \
  -p frame_id:=velodyne \
  -p model:=VLP16 \
  -p port:=2368 \
  -p rpm:=600.0 \
  -p gps_time:=false \
  -p read_fast:=false \
  -p read_once:=false \
  -p repeat_delay:=0.0 \
  -p timestamp_first_packet:=false

# トピック確認
ros2 topic echo /velodyne_points --no-arr | head -20

# 終了時
docker compose down
```

#### 2台PC間検証（velodyne_driverを使用）

```bash
# --- シミュレーションPC ---
# USB-LANアダプタにVLP-16のIPを割当
sudo ip addr add 192.168.1.201/24 dev eth1

# エミュレータ起動（Gazebo等のPointCloud2をサブスクライブ）
ros2 launch hils_bridge_lidar_velodyne velodyne_emulator.launch.py \
  network_interface:=eth1 \
  host_ip:=192.168.1.100 \
  pointcloud_topic:=/points

# --- 実機PC (192.168.1.100) ---
# velodyne_driverを起動
ros2 launch velodyne_driver velodyne_driver_node-VLP16-launch.py \
  device_ip:=192.168.1.201

# トピック確認
ros2 topic echo /velodyne_points --no-arr | head -20
```

### 1.3 Ouster OS1

Velodyneと同様の手順。ポートは7502（LiDAR）/ 7503（IMU）。

```bash
# シミュレーションPC
sudo ip addr add 192.168.1.100/24 dev eth1
ros2 launch hils_bridge_lidar_ouster ouster_emulator.launch.py \
  network_interface:=eth1 \
  host_ip:=192.168.1.5

# 実機PC
# ouster_ros ドライバを起動
```

---

## 2. シリアルセンサエミュレータ（2台PC + FT234X × 2）

### 必要機材

- FT234X超小型USBシリアル変換モジュール × 2（秋月 [108461]）
- ジャンパワイヤ 2本（TX/RXクロス接続用）

### 配線

```
FT234X #1 (シミュPC側)      FT234X #2 (実機PC側)
  TX ──────────────────────── RX
  RX ──────────────────────── TX
  GND ─────────────────────── GND
```

### 2.1 GPS NMEA

```bash
# --- シミュレーションPC ---
# FT234X #1 のデバイスパスを確認
ls /dev/ttyUSB*

# GPSブリッジ起動
ros2 launch hils_bridge_serial_gps gps_bridge.launch.py \
  serial_port:=/dev/ttyUSB0 \
  fix_topic:=/gps/fix

# テスト用NavSatFixパブリッシュ
ros2 topic pub /gps/fix sensor_msgs/NavSatFix \
  "{'header': {'stamp': {'sec': 1713400000}}, 'status': {'status': 0}, 'latitude': 35.6812, 'longitude': 139.7671, 'altitude': 40.0}" \
  --rate 1

# --- 実機PC ---
# FT234X #2 のデバイスパスを確認
ls /dev/ttyUSB*

# NMEAデータの生着確認（ドライバなしで直接確認）
cat /dev/ttyUSB0
# → $GPGGA,... と $GPRMC,... が表示されるはず

# nmea_navsat_driverで確認
ros2 launch nmea_navsat_driver nmea_serial_driver_node.launch.py \
  port:=/dev/ttyUSB0 \
  baud:=9600

ros2 topic echo /fix
```

### 2.2 シリアルIMU (Witmotion WT901)

```bash
# --- シミュレーションPC ---
ros2 launch hils_bridge_serial_imu imu_bridge.launch.py \
  serial_port:=/dev/ttyUSB0 \
  imu_topic:=/imu/data

# テスト用Imuパブリッシュ
ros2 topic pub /imu/data sensor_msgs/Imu \
  "{'header': {'frame_id': 'imu'}, 'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.8}, 'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.1}}" \
  --rate 50

# --- 実機PC ---
# バイナリデータの生着確認
xxd /dev/ttyUSB0 | head -20
# → 55 51 ... 55 52 ... 55 53 ... のパターンが見えるはず

# witmotion_ros2ドライバで確認（ドライバがインストールされている場合）
```

---

## 3. UVCカメラエミュレータ（実機PC + RP2040 × 2）

### 必要機材

- Raspberry Pi Pico H × 2
- USBケーブル (A-microB) × 2
- ジャンパワイヤ（Pico間UART接続用）

### 配線

```
Pico#1 (CDC-SPI sender)          Pico#2 (UVC bridge)
  GPIO 0 (TX) ──────────────────── GPIO 1 (RX)
  GPIO 1 (RX) ──────────────────── GPIO 0 (TX)
  GND ──────────────────────────── GND
```

### 手順

```bash
# 1. ファームウェアをフラッシュ（BOOTSELボタンを押しながら接続）
cp firmware/rp2040_cdc_spi_sender/build/rp2040_cdc_spi_sender.uf2 /media/$USER/RPI-RP2/
cp firmware/rp2040_uvc_bridge/build/rp2040_uvc_bridge.uf2 /media/$USER/RPI-RP2/

# 2. シミュレーションPC: UVCブリッジ起動
ros2 launch hils_bridge_camera_uvc uvc_bridge.launch.py \
  serial_port:=/dev/ttyACM0 \
  image_topic:=/image_raw

# 3. 実機PC: UVCカメラとして認識されているか確認
v4l2-ctl --list-devices
# → Pico#2 が /dev/video* として表示

# 4. 実機PC: 映像確認
ffplay /dev/video0
# または
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
ros2 topic echo /image_raw --no-arr | head -5
```

---

## 4. PWMサーボ + エンコーダエミュレータ（PC + RP2040）

### 必要機材

- Raspberry Pi Pico H × 1
- USBケーブル (A-microB) × 1
- オシロスコープまたはロジックアナライザ（PWM波形確認用）

### 配線

```
RP2040 PWM Servo Emulator
  GPIO 2: Servo CH0 PWM出力
  GPIO 3: Servo CH1 PWM出力
  GPIO 4: Servo CH2 PWM出力
  GPIO 5: Servo CH3 PWM出力
  GPIO 6: Encoder CH0 A相
  GPIO 7: Encoder CH0 B相
  GPIO 8: Encoder CH1 A相
  GPIO 9: Encoder CH1 B相
```

### 手順

```bash
# 1. ファームウェアをフラッシュ
cp firmware/rp2040_pwm_servo/build/rp2040_pwm_servo.uf2 /media/$USER/RPI-RP2/

# 2. シミュレーションPC: PWMブリッジ起動
ros2 launch hils_bridge_actuator_pwm pwm_bridge.launch.py \
  serial_port:=/dev/ttyACM0

# 3. テスト用JointStateパブリッシュ
ros2 topic pub /joint_states sensor_msgs/JointState \
  "{'name': ['joint0', 'joint1'], 'position': [0.0, 1.57]}" \
  --rate 50

# 4. オシロスコープでGPIO 2-5のPWM波形を確認
#    joint0: 0.0 rad → 1500us (センター)
#    joint1: 1.57 rad (≈π/2) → 2500us (最大)

# 5. ロジックアナライザでGPIO 6-7のエンコーダパルスを確認
#    A/B相のクワドラチャ信号が出力されるはず
```

---

## 5. I2Cセンサエミュレータ（PC + RP2040）

### 必要機材

- Raspberry Pi Pico H × 1
- USBケーブル (A-microB) × 1
- I2Cマスターデバイス（Arduino、別のPico等）
- ジャンパワイヤ、プルアップ抵抗 4.7kΩ × 2

### 配線

```
RP2040 I2C Slave                   I2Cマスター (Arduino等)
  GPIO 4 (SDA) ───────┬──────────── SDA
                      R 4.7kΩ
                      │
                     3.3V
  GPIO 5 (SCL) ───────┬──────────── SCL
                      R 4.7kΩ
                      │
                     3.3V
  GND ─────────────────────────── GND
```

### 手順

```bash
# 1. ファームウェアをフラッシュ
cp firmware/rp2040_i2c_slave/build/rp2040_i2c_slave.uf2 /media/$USER/RPI-RP2/

# 2. シミュレーションPC: I2Cセンサブリッジ起動
ros2 launch hils_bridge_sensor_i2c i2c_sensor_bridge.launch.py \
  serial_port:=/dev/ttyACM0

# 3. テスト用Imuパブリッシュ
ros2 topic pub /imu/data sensor_msgs/Imu \
  "{'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.8}}" \
  --rate 100

# 4. I2Cマスター側でMPU-6050を読み取り
#    Arduino: Wire.requestFrom(0x68, 14) でレジスタ 0x3B-0x48 を読み出し
#    WHO_AM_I (0x75) = 0x68 が返ること
#    ACCEL_ZOUT = 16384 前後（1g = 16384 LSB）
```

---

## 検証優先順位

最も手軽に検証できる順：

1. **Livox Mid-360** — 既に動作確認済み
2. **Velodyne VLP-16 / Ouster OS1** — USB-LANアダプタ or dummyインターフェースで即検証可能
3. **GPS NMEA / IMU** — FT234X × 2 が必要（要購入）
4. **UVCカメラ** — Pico × 2 + ファームウェアビルド済みなら即検証可能
5. **PWMサーボ** — Pico × 1 + オシロスコープ推奨
6. **I2C IMU** — Pico × 1 + I2Cマスター + プルアップ抵抗

LiDAR系は1台PCで即座に試せるので、まずそこから始めるのが効率的。
