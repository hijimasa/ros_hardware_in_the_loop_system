[English](README.md) | **日本語**

# ROS Hardware-in-the-Loop Simulation System

安価なマイコンと汎用 USB デバイスを活用して、ROS 2 ロボットの **物理通信経路を含む Hardware-in-the-Loop Simulation (HILS)** を実現するプロジェクト。

UVCカメラの動作確認（Isaac Simからのシミュレーション映像を伝送）
![UVCカメラの動作確認](./figs/uvc_camera_hils.gif)

Livox MID360の動作確認（ros2 bagのデータをトピックではなく元のプロトコルで伝送）
![Livox MID360の動作確認](./figs/livox_mid360_hils.gif)

## 概要

ソフトウェアシミュレーション（Gazebo / Unity / Isaac Sim）ではセンサドライバや物理通信経路がバイパスされる。本プロジェクトでは、シミュレータの出力を実機センサのプロトコル（UDP, UART, USB UVC, I2C 等）に変換し、実機と同一のドライバスタックを使ってテストする HILS 環境を **数千円の部品代** で構築する。

## リポジトリ構成

```
ros_hardware_in_the_loop_system/
├── ros2_hils_bridge/          # ROS 2 パッケージ群 (サブモジュール)
├── firmware/                  # RP2040/ESP32 ファームウェア
│   ├── common/                #   共通ヘッダ (HILS フレームプロトコル)
│   ├── rp2040_uvc_bridge/     #   UVC カメラエミュレータ
│   ├── rp2040_cdc_spi_sender/ #   CDC-SPI 中継 (カメラ用)
│   ├── rp2040_pwm_servo/      #   PWM サーボ + エンコーダ出力
│   ├── rp2040_i2c_slave/      #   I2C スレーブ (MPU-6050)
│   ├── rp2040_encoder_emulator/ # エンコーダ出力 (単体)
│   ├── rp2040_ethernet_bridge/  # Ethernet ブリッジ (参考実装)
│   └── esp32_can_bridge/      #   CAN ブリッジ (参考実装)
├── docs/                      # アーキテクチャ、BOM、検証手順
└── hardware/                  # 回路図、筐体 (予定)
```

### サブモジュール: ros2_hils_bridge

ROS 2 パッケージ群は `ros2_hils_bridge/` に分離されており、`colcon_ws/src` に直接クローンしてビルドできる。詳細は [ros2_hils_bridge/README.md](ros2_hils_bridge/README.md) を参照。

## 実装状態

### 方面 A: PC 開発者向け（マイコン不要）

| デバイス | 方式 | ROS パッケージ | ファームウェア | 状態 |
|---------|------|--------------|-------------|------|
| Livox Mid-360 | USB-LAN + SW | hils_bridge_lidar_livox | 不要 | 実装済・**動作確認済** |
| Velodyne VLP-16 | USB-LAN + SW | hils_bridge_lidar_velodyne | 不要 | 実装済・未検証 |
| Ouster OS1 | USB-LAN + SW | hils_bridge_lidar_ouster | 不要 | 実装済・未検証 |
| GPS (NMEA) | FT234X x 2 | hils_bridge_serial_gps | 不要 | 実装済・未検証 |
| IMU (Witmotion) | FT234X x 2 | hils_bridge_serial_imu | 不要 | 実装済・未検証 |

### 方面 B: マイコン開発者向け（RP2040 ファームウェア）

| デバイス | 方式 | ROS パッケージ | ファームウェア | 状態 |
|---------|------|--------------|-------------|------|
| USB カメラ | RP2040 UVC | hils_bridge_camera_uvc | rp2040_uvc_bridge + rp2040_cdc_spi_sender | 実装済・**動作確認済** |
| PWM サーボ | RP2040 PIO | hils_bridge_actuator_pwm | rp2040_pwm_servo | 実装済・未検証 |
| エンコーダ | RP2040 PIO | (pwm と統合) | rp2040_pwm_servo | 実装済・未検証 |
| I2C IMU (MPU-6050) | RP2040 I2C slave | hils_bridge_sensor_i2c | rp2040_i2c_slave | 実装済・未検証 |

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
cd firmware/rp2040_uvc_bridge
mkdir build && cd build
cmake .. -DPICO_SDK_PATH=~/pico-sdk
make -j$(nproc)
# build/rp2040_uvc_bridge.uf2 を BOOTSEL モードの Pico にコピー
```

## ドキュメント

- [アーキテクチャ設計](docs/ros_hils_architecture.md)
- [部品リスト (BOM)](docs/hils_verification_bom.md)
- [拡張ロードマップ](docs/hils_expansion_roadmap.md)
- [動作確認手順書](docs/hils_verification_guide.md)

## ライセンス

MIT
