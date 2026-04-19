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
ros2 launch hils_bridge_lidar_velodyne_vlp16 velodyne_emulator.launch.py \
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
ros2 launch hils_bridge_lidar_velodyne_vlp16 velodyne_emulator.launch.py \
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

Velodyneと同様の手順だが、**Ouster 固有の注意点が多い**ので以下を守ること。

#### ⚠️ Ouster 固有の必須事項

| 項目 | 内容 | 理由 |
|------|------|------|
| **sim コンテナの sysctl** | `net.ipv4.ip_unprivileged_port_start=80` を設定 | エミュレータは HTTP REST API を port 80 で提供。非root ユーザーが bind するため必要 |
| **firmware_ver ≥ 2.4** | エミュレータは `v2.5.2` を返す | modern な ouster_ros は 2.4 以上しか受け付けない |

> **NOTE:** 以前は `lidar_port:=7502 imu_port:=7503` の明示指定が必要だったが、`set_config_param` クエリの URL decode の副作用を修正したことで `lidar_port:=0`（デフォルト=random port）でも動的追従が効くようになった。両方の方式で動作確認済み。

#### 1台PC内検証（Docker Compose で 2 コンテナ）

```bash
cd docker
docker compose up -d

# sim コンテナ: エミュレータを起動
docker compose exec sim bash
source /opt/ros/jazzy/setup.bash
cd ~/colcon_ws && colcon build && source install/setup.bash
ros2 launch hils_bridge_lidar_ouster_os1 ouster_emulator.launch.py \
  device_ip:=192.168.100.201 \
  host_ip:=192.168.100.100 \
  network_interface:=eth0 \
  pointcloud_topic:=/livox/lidar   # 入力トピック名は環境に合わせる

# 別ターミナルで robot コンテナ: ドライバを起動
docker compose exec robot bash
source /opt/ros/jazzy/setup.bash
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=192.168.100.201 \
  viz:=false
# デフォルト (lidar_port:=0 imu_port:=0 = random) で動作する。
# 明示指定したい場合は lidar_port:=7502 imu_port:=7503 を追加。

# 別ターミナルで確認
docker compose exec robot bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /ouster/points
# 期待: 約10Hz
ros2 topic echo --once /ouster/points | head -20
# 期待: height=16, width=512 (OS1-16 × 512x10mode)
```

#### 2台PC間検証

```bash
# --- シミュレーションPC ---
sudo ip addr add 192.168.1.100/24 dev eth1
ros2 launch hils_bridge_lidar_ouster_os1 ouster_emulator.launch.py \
  network_interface:=eth1 \
  host_ip:=192.168.1.5 \
  pointcloud_topic:=/points

# --- 実機PC (192.168.1.5) ---
ros2 launch ouster_ros sensor.launch.xml \
  sensor_hostname:=192.168.1.100 \
  viz:=false
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

# GPSブリッジ起動（fix と vel の両方を subscribe する）
ros2 launch hils_bridge_gps_nmea0183 gps_bridge.launch.py \
  serial_port:=/dev/ttyUSB0 \
  fix_topic:=/gps/fix
  # vel_topic:=/gps/vel  # デフォルト。enable_velocity:=false で無効化可

# テスト用NavSatFixパブリッシュ
ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix \
  "{'header': {'stamp': {'sec': 1713400000}}, 'status': {'status': 0}, 'latitude': 35.6812, 'longitude': 139.7671, 'altitude': 40.0}" \
  --rate 1

# 別ターミナルで TwistStamped をパブリッシュ（GPRMC の speed/course を検証）
# 北東方向に約 10 m/s ≒ 19.4 knot で移動するシナリオ
ros2 topic pub /gps/vel geometry_msgs/msg/TwistStamped \
  "{'header': {'frame_id': 'gps'}, 'twist': {'linear': {'x': 7.07, 'y': 7.07, 'z': 0.0}}}" \
  --rate 1

# --- 実機PC ---
# FT234X #2 のデバイスパスを確認
ls /dev/ttyUSB*

# NMEAデータの生着確認（ドライバなしで直接確認）
cat /dev/ttyUSB0
# → $GPGGA,... と $GPRMC,... が表示されるはず
# → GPRMC の speed フィールドが空でなく、course も入っていること

# nmea_navsat_driver で確認
# NOTE: 上流の nmea_serial_driver.launch.py は port/baud 引数を無視するため
#       ros2 run で --ros-args 指定する方法を使う
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baud:=9600

# /fix と /vel の両方を確認
ros2 topic echo --once /fix
# → latitude/longitude がエミュレータ側 /gps/fix と一致
ros2 topic echo --once /vel
# → twist.linear.x/y が NaN ではなく、北東方向の速度成分が入っていること
#   （/gps/vel を pub していない場合は NaN になる = 想定通り）
```

### 2.2 シリアルIMU (Witmotion WT901)

```bash
# --- シミュレーションPC ---
ros2 launch hils_bridge_imu_witmotion_wt901 imu_bridge.launch.py \
  serial_port:=/dev/ttyUSB0 \
  imu_topic:=/imu/data

# テスト用Imuパブリッシュ
ros2 topic pub /imu/data sensor_msgs/Imu \
  "{'header': {'frame_id': 'imu'}, 'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.8}, 'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.1}}" \
  --rate 50

# --- 実機PC ---
# バイナリデータの生着確認
# NOTE: cat/od/xxd は tty を canonical mode で開くため、
#       事前に raw + 正しい baud に切り替える必要がある
stty -F /dev/ttyUSB1 115200 raw -echo -echoe -echok
od -An -tx1 -N66 /dev/ttyUSB1
# → 55 51 ... 55 52 ... 55 53 ... のパターンが見えるはず

# witmotion_ros (ElettraSciComp版) で確認
# NOTE: 上流の wt901.yml デフォルト port は ttyUSB0 なので変更が必要。
#       use_native_orientation: true (デフォルト) のままで OK
#       (エミュレータは 0x59 Quaternion パケットも送るため)。
#       姿勢推定なしのシナリオで Euler→Quat 変換に切り替えたい場合のみ
#       use_native_orientation: false に変更する。
sudo sed -i 's|port: .*|port: /dev/ttyUSB1|' \
  $(ros2 pkg prefix witmotion_ros)/share/witmotion_ros/config/wt901.yml

ros2 launch witmotion_ros wt901.launch.py

ros2 topic hz /imu
# 期待: 約25Hz（emulator 50Hz × 3pkt を 50ms polling で消費するため）
ros2 topic echo --once /imu
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
cp firmware/rp2040_camera_uvc_spi_sender/build/rp2040_camera_uvc_spi_sender.uf2 /media/$USER/RPI-RP2/
cp firmware/rp2040_camera_uvc/build/rp2040_camera_uvc.uf2 /media/$USER/RPI-RP2/

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

## 4. RC サーボ PWM エミュレータ（PC + RP2040）

PWM サーボ出力（コマンド方向）と クワドラチャエンコーダ出力（フィードバック方向）は別々の firmware／ROS パッケージに分離されている。それぞれ独立した Pico に書き込んで使う。

### 4.1 サーボ PWM 出力

#### 必要機材

- Raspberry Pi Pico H × 1
- USBケーブル (A-microB) × 1
- オシロスコープ（PWM波形確認用）

#### 配線

```
RP2040 (rp2040_actuator_servo_pwm)
  GPIO 2: Servo CH0 PWM出力
  GPIO 3: Servo CH1 PWM出力
  GPIO 4: Servo CH2 PWM出力
  GPIO 5: Servo CH3 PWM出力
```

#### 手順

```bash
# 1. ファームウェアをフラッシュ
cp firmware/rp2040_actuator_servo_pwm/build/rp2040_actuator_servo_pwm.uf2 /media/$USER/RPI-RP2/

# 2. シミュレーションPC: PWM ブリッジ起動
ros2 launch hils_bridge_actuator_servo_pwm pwm_bridge.launch.py \
  serial_port:=/dev/ttyACM0

# 3. テスト用 JointState パブリッシュ
ros2 topic pub /joint_states sensor_msgs/JointState \
  "{'name': ['joint0', 'joint1'], 'position': [0.0, 1.57]}" \
  --rate 50

# 4. オシロスコープで GPIO 2-5 の PWM 波形を確認
#    joint0: 0.0 rad → 1500us (センター)
#    joint1: 1.57 rad (≈π/2) → 2500us (最大)
```

### 4.2 クワドラチャエンコーダ出力

#### 必要機材

- Raspberry Pi Pico H × 1（4.1 とは別の Pico）
- USBケーブル (A-microB) × 1
- ロジックアナライザ

#### 配線

```
RP2040 (rp2040_encoder_quadrature)
  GPIO 6: Encoder CH0 A相
  GPIO 7: Encoder CH0 B相
  GPIO 8: Encoder CH1 A相
  GPIO 9: Encoder CH1 B相
```

#### 手順

```bash
# 1. ファームウェアをフラッシュ
cp firmware/rp2040_encoder_quadrature/build/rp2040_encoder_quadrature.uf2 /media/$USER/RPI-RP2/

# 2. シミュレーションPC: エンコーダブリッジ起動
ros2 launch hils_bridge_encoder_quadrature encoder_bridge.launch.py \
  serial_port:=/dev/ttyACM0 encoder_cpr:=1000

# 3. テスト用 JointState パブリッシュ
ros2 topic pub /joint_states sensor_msgs/JointState \
  "{'name': ['joint0', 'joint1'], 'position': [0.0, 1.57]}" \
  --rate 50

# 4. ロジックアナライザで GPIO 6-7 (CH0) と GPIO 8-9 (CH1) を観測
#    joint1=1.57rad, cpr=1000 → 約 250 カウント分の A/B 相パルスが流れる
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
cp firmware/rp2040_imu_invensense_mpu6050/build/rp2040_imu_invensense_mpu6050.uf2 /media/$USER/RPI-RP2/

# 2. シミュレーションPC: I2Cセンサブリッジ起動
ros2 launch hils_bridge_imu_invensense_mpu6050 i2c_sensor_bridge.launch.py \
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
