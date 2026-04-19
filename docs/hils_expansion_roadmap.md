# HILS 拡張ロードマップ

> **状態: Phase 0〜2 全て完了** (2026-04-18)
> 全25パッケージがビルド成功。動作確認フェーズに移行。

## 概要

UVCカメラ（RP2040）およびLivox Mid-360（純ソフトウェア）のHILSが安定動作したことを受け、対応デバイスを拡張する。2つの方面で並行して進める。

---

## 2方面戦略

### 方面A: PC開発者向け（純ソフトウェア、マイコン不要）

USBシリアル接続・LAN接続のデバイスプロトコルを拡充し、PCベースのROS2ロボット開発者が安価にHILSを導入できるようにする。

| 優先度 | ターゲット | 方式 | ハードウェア | 依存 |
|--------|-----------|------|------------|------|
| 優先度 | ターゲット | 方式 | ハードウェア | 状態 |
|--------|-----------|------|------------|------|
| **A1** | GPS (NMEA) | FT234X × 2 クロス接続 | FT234X 800円 | **実装済** |
| **A2** | Velodyne VLP-16 | USB-LANアダプタ + UDP | USB-LANアダプタ ~1,000円 | **実装済** |
| **A3** | シリアルIMU (Witmotion WT901) | FT234X × 2 クロス接続 | FT234X 800円 | **実装済** |
| **A4** | Ouster OS1 | USB-LANアダプタ + UDP | USB-LANアダプタ ~1,000円 | **実装済** |

### 方面B: マイコン開発者向け（RP2040ファームウェア）

PWM/I2C/SPIインターフェースを模擬し、Arduino/RP2040でロボットを開発するユーザーにもHILSを訴求する。

| 優先度 | ターゲット | 方式 | ハードウェア | 依存 |
|--------|-----------|------|------------|------|
| 優先度 | ターゲット | 方式 | ハードウェア | 状態 |
|--------|-----------|------|------------|------|
| **B1** | PWMサーボ（取り込み/評価） | RP2040 PIO パルス幅計測 | Pico H 920円 | **実装済・動作確認済** |
| **B2** | エンコーダ出力 | RP2040 PIO A/B相出力 | B1と同一Pico | **実装済** |
| **B3** | I2C IMU (MPU-6050) | RP2040 HW I2Cスレーブ | Pico H 920円 | **実装済** |
| **B4** | SPI距離センサ (VL53L0X等) | RP2040 SPIスレーブ | B3と同一Pico | 未実装（B3のパターンで横展開可能） |

---

## 実行フェーズ

### Phase 0: 共通基盤整備

両方面の前提となる共通モジュールを `hils_bridge_base` パッケージに整備する。

| モジュール | 内容 | 移動元/新規 |
|-----------|------|------------|
| `frame_protocol.py` | フレームプロトコル ビルダー/パーサー | camera_uvc から移動 |
| `network_utils.py` | IP検出・インターフェースバインド・サブネット検証 | livox_emulator_node から抽出 |
| `serial_bridge_base.py` | FT234Xクロス接続向けシリアルブリッジ基底クラス | 新規 |
| `udp_emulator_base.py` | UDP機器エミュレータ基底クラス（ソケット管理、レート制御、統計） | livox_emulator_node から抽出 |

**既存ノードの更新:**
- `livox_emulator_node.py` → `network_utils`, `udp_emulator_base` を利用
- `uvc_bridge_node.py` → `frame_protocol` を `hils_bridge_base` から import

### Phase 1: 並行開発（3エージェント）

Phase 0完了後、以下を並行して進められる：

```
Agent 1: GPS NMEA bridge (A1)
  - hils_bridge_gps/hils_bridge_gps_nmea0183/
  - serial_bridge_base を継承
  - NMEAセンテンス生成 ($GPGGA, $GPRMC, $GPVTG)
  - NavSatFix → NMEA変換

Agent 2: Velodyne VLP-16 emulator (A2)
  - hils_bridge_lidar/hils_bridge_lidar_velodyne_vlp16/
  - udp_emulator_base を継承
  - VLP-16パケットフォーマット (port 2368/8308)
  - PointCloud2 → Velodyne UDP変換

Agent 3: RC サーボ PWM (B1) + クワドラチャエンコーダ (B2) — 別 firmware／別 ROS パッケージ
  - firmware/rp2040_actuator_servo_pwm/  + hils_bridge_actuator/hils_bridge_actuator_servo_pwm/
  - firmware/rp2040_encoder_quadrature/ + hils_bridge_encoder/hils_bridge_encoder_quadrature/
  - PIOプログラム + CDCブリッジノード
```

### Phase 2: 横展開

Phase 1のパターンを流用して残りのデバイスに横展開：

- A3 (シリアルIMU): A1のserial_bridge_baseパターンを利用、バイナリプロトコル対応
- A4 (Ouster OS1): A2のudp_emulator_baseパターンを利用
- B3 (I2C IMU): B1/B2のファームウェアパターンを利用、PIO I2Cスレーブ
- B4 (SPI距離センサ): B3と同様

---

## パッケージ構成（完成時）

```
ros2_hils_bridge/
├── hils_bridge_base/                          # 共通基盤 (Phase 0)
│   └── hils_bridge_base/
│       ├── frame_protocol.py
│       ├── network_utils.py
│       ├── serial_bridge_base.py
│       └── udp_emulator_base.py
├── hils_bridge_camera/
│   └── hils_bridge_camera_uvc/                # 実装済み
├── hils_bridge_lidar/
│   ├── hils_bridge_lidar_livox_mid360/        # 実装済み
│   ├── hils_bridge_lidar_velodyne_vlp16/      # 実装済み (A2)
│   └── hils_bridge_lidar_ouster_os1/          # 実装済み (A4)
├── hils_bridge_gps/
│   └── hils_bridge_gps_nmea0183/              # 実装済み (A1)
├── hils_bridge_imu/
│   ├── hils_bridge_imu_witmotion_wt901/       # 実装済み (A3)
│   └── hils_bridge_imu_invensense_mpu6050/    # 実装済み (B3)
├── hils_bridge_actuator/
│   └── hils_bridge_actuator_servo_pwm/        # 実装済み (B1)
├── hils_bridge_encoder/
│   └── hils_bridge_encoder_quadrature/        # 実装済み (B2)
└── hils_bridge_can/                           # placeholder

firmware/
├── rp2040_camera_uvc/                        # 実装済み
├── rp2040_camera_uvc_spi_sender/             # 実装済み
├── rp2040_actuator_servo_pwm/                # 実装済み (B1)
├── rp2040_encoder_quadrature/                # 実装済み (B2)
└── rp2040_imu_invensense_mpu6050/            # 実装済み (B3)
```
