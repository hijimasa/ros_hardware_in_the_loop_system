[English](README.md) | **日本語**

# ROS Hardware-in-the-Loop Simulation System

安価なマイコンと汎用 USB デバイスを活用して、ROS 2 ロボットの **物理通信経路を含む Hardware-in-the-Loop Simulation (HILS)** を実現するプロジェクト。

UVCカメラの動作確認（Isaac Simからのシミュレーション映像を伝送）
![UVCカメラの動作確認](./figs/uvc_camera_hils.gif)

Livox MID360の動作確認（ros2 bagのデータをトピックではなく元のプロトコルで伝送）
![Livox MID360の動作確認](./figs/livox_mid360_hils.gif)

RC サーボ PWM 取り込みの動作確認（Arduino 制御器が出力する PWM を計測して JointState としてパブリッシュ）
![RC サーボ PWM HILS 動作確認](./figs/pwm_servo_hils.gif)

## 概要

ソフトウェアシミュレーション（Gazebo / Unity / Isaac Sim）ではセンサドライバや物理通信経路がバイパスされる。本プロジェクトでは、シミュレータの出力を実機センサのプロトコル（UDP, UART, USB UVC, I2C 等）に変換し、実機と同一のドライバスタックを使ってテストする HILS 環境を **数千円の部品代** で構築する。

## リポジトリ構成

```
ros_hardware_in_the_loop_system/
├── ros2_hils_bridge/                     # ROS 2 パッケージ群 (サブモジュール)
├── firmware/                             # RP2040 / ESP32 ファームウェア
│   ├── common/                           #   共通ヘッダ (HILS フレームプロトコル)
│   ├── rp2040_camera_uvc/                #   UVC カメラ (Pico#2)
│   ├── rp2040_camera_uvc_spi_sender/     #   USB-CDC -> SPI 中継 (Pico#1)
│   ├── rp2040_actuator_servo_pwm/        #   RC サーボ PWM 取り込み（制御器出力の計測・評価）
│   ├── rp2040_encoder_quadrature/        #   クワドラチャエンコーダ A/B 出力
│   ├── rp2040_imu_invensense_mpu6050/    #   I2C スレーブ MPU-6050 レジスタマップ
│   ├── rp2040_ethernet_bridge/           #   Ethernet ブリッジ (参考実装)
│   └── esp32_can_bridge/                 #   CAN ブリッジ (参考実装)
├── docs/                                 # アーキテクチャ、BOM、検証手順
└── hardware/                             # 回路図、筐体 (予定)
```

### サブモジュール: ros2_hils_bridge

ROS 2 パッケージ群は `ros2_hils_bridge/` に分離されており、`colcon_ws/src` に直接クローンしてビルドできる。詳細は [ros2_hils_bridge/README.md](ros2_hils_bridge/README.md) を参照。

## 命名規則

新しいエミュレータを追加する際にぶれないよう、パッケージ・ファームウェアともに **2 段階の命名パターン** を採用している：

```
ros2_hils_bridge/hils_bridge_<sensor_type>/hils_bridge_<sensor_type>_<protocol_or_vendor_series>/
firmware/rp2040_<sensor_type>_<protocol_or_vendor_series>/
```

- **`<sensor_type>`** — 物理／機能カテゴリ: `lidar`, `camera`, `gps`, `imu`, `actuator`, `encoder`, `can`
- **`<protocol_or_vendor_series>`** — 以下のルールで選ぶ：
  - **業界標準プロトコル** (UVC, NMEA0183, PWM, quadrature 等) → プロトコル名そのものを使う。任意のベンダーで使い回せることを示す (例: `hils_bridge_camera_uvc`, `hils_bridge_gps_nmea0183`)
  - **ベンダー固有プロトコル** → `<ベンダー>_<シリーズ>` 形式。パッケージの実装範囲を曖昧にしない (例: `hils_bridge_lidar_livox_mid360`, `hils_bridge_imu_witmotion_wt901`, `hils_bridge_imu_invensense_mpu6050`)。同一ベンダーでも世代やシリーズで非互換になりがちなので必ずシリーズ名まで含める

新規追加時はまずセンサ種別を決め、対応プロトコルが業界標準なら標準名を、ベンダー固有なら `<ベンダー>_<シリーズ>` を選ぶ。ROS パッケージ名と対応する firmware ディレクトリは常に一致させる。

## 実装状態

### 方面 A: PC 開発者向け（マイコン不要）

| デバイス | 方式 | ROS パッケージ | ファームウェア | 状態 |
|---------|------|--------------|-------------|------|
| Livox Mid-360 | USB-LAN + SW | hils_bridge_lidar_livox_mid360 | 不要 | 実装済・**動作確認済** |
| Velodyne VLP-16 | USB-LAN + SW | hils_bridge_lidar_velodyne_vlp16 | 不要 | 実装済・**動作確認済** |
| Ouster OS1 | USB-LAN + SW | hils_bridge_lidar_ouster_os1 | 不要 | 実装済・**動作確認済**[^ouster-note] |
| GPS (NMEA 0183) | FT234X x 2 | hils_bridge_gps_nmea0183 | 不要 | 実装済・**動作確認済** |
| IMU (Witmotion WT901) | FT234X x 2 | hils_bridge_imu_witmotion_wt901 | 不要 | 実装済・**動作確認済**[^witmotion-note] |

[^ouster-note]: Ouster はエミュレータ側で HTTP REST API (port 80) を提供するため Docker では `sysctls: net.ipv4.ip_unprivileged_port_start=80` が必要。詳細は [docs/hils_verification_guide.md](docs/hils_verification_guide.md#13-ouster-os1) を参照。
[^witmotion-note]: IMU エミュレータは WT901 標準の 4 種パケット (0x51 加速度 / 0x52 角速度 / 0x53 オイラー / 0x59 クォータニオン) を出力するため、`witmotion_ros` (ElettraSciComp) のデフォルト `use_native_orientation: true` のまま動作する。ドライバビルドには `libqt5serialport5-dev` が必要（Dockerfile 反映済）。詳細は [docs/hils_verification_guide.md](docs/hils_verification_guide.md#22-シリアルimu-witmotion-wt901) を参照。

### 方面 B: マイコン開発者向け（RP2040 ファームウェア）

| デバイス | 方式 | ROS パッケージ | ファームウェア | 状態 |
|---------|------|--------------|-------------|------|
| USB カメラ | RP2040 UVC | hils_bridge_camera_uvc | rp2040_camera_uvc + rp2040_camera_uvc_spi_sender | 実装済・**動作確認済** |
| RC サーボ（取り込み） | RP2040 PIO パルス幅計測 | hils_bridge_actuator_servo_pwm | rp2040_actuator_servo_pwm | 実装済・**動作確認済**（Arduino PWM → JointState） |
| クワドラチャエンコーダ | RP2040 PIO | hils_bridge_encoder_quadrature | rp2040_encoder_quadrature | 実装済・未検証 |
| I2C IMU (MPU-6050) | RP2040 I2C slave | hils_bridge_imu_invensense_mpu6050 | rp2040_imu_invensense_mpu6050 | 実装済・未検証 |

## クイックスタート

### ROS 2 パッケージのビルド

```bash
cd ~/colcon_ws/src
git clone --recursive https://github.com/<your-org>/ros_hardware_in_the_loop_system.git
# または ros2_hils_bridge のみ:
# git clone https://github.com/<your-org>/ros2_hils_bridge.git

cd ~/colcon_ws
colcon build
source install/setup.bash
```

### ファームウェアのビルド（RP2040 使用時のみ）

```bash
cd firmware/rp2040_camera_uvc
mkdir build && cd build
cmake .. -DPICO_SDK_PATH=~/pico-sdk
make -j$(nproc)
# build/rp2040_camera_uvc.uf2 を BOOTSEL モードの Pico にコピー
```

## ドキュメント

- [アーキテクチャ設計](docs/ros_hils_architecture.md)
- [部品リスト (BOM)](docs/hils_verification_bom.md)
- [拡張ロードマップ](docs/hils_expansion_roadmap.md)
- [動作確認手順書](docs/hils_verification_guide.md)

## ライセンス

MIT
