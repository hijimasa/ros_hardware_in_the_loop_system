# W5500 Livox Mid360 Emulator Firmware

W5500-EVB-Pico2 上で動作する Livox Mid360 エミュレータファームウェア。
USB CDC 経由で受信した点群・IMU データを、Livox SDK2 プロトコルに変換し
W5500 Ethernet から送信する。ロボット PC 上の `livox_ros_driver2` がそのまま使用可能。

## データフロー

```
Simulation PC                            W5500-EVB-Pico2                    Robot PC
┌────────────────┐     USB CDC      ┌──────────────────┐    Ethernet    ┌────────────────┐
│ hils_bridge_   │  HILS frame      │ w5500_livox_     │  Livox SDK2   │ livox_ros_     │
│ lidar_livox    │ ──────────────>  │ emulator         │ ───────────>  │ driver2        │
│ (ROS2 node)    │  (点群/IMU/設定)  │ (this firmware)  │  (UDP)        │ (ROS2 driver)  │
└────────────────┘                  └──────────────────┘               └────────────────┘
```

## ハードウェア

### W5500-EVB-Pico2 ピン配置

| 機能 | ピン | 説明 |
|------|------|------|
| SPI0 SCK | GP18 | W5500 SPI クロック |
| SPI0 MOSI | GP19 | W5500 SPI データ出力 |
| SPI0 MISO | GP16 | W5500 SPI データ入力 |
| SPI0 CS | GP17 | W5500 チップセレクト |
| W5500 RST | GP20 | W5500 ハードウェアリセット |
| W5500 INT | GP21 | W5500 割り込み (未使用) |
| UART0 TX | GP0 | デバッグ出力 |
| UART0 RX | GP1 | デバッグ入力 |
| LED | GP25 | ステータスLED |

### LED パターン

| パターン | 状態 |
|---------|------|
| 高速点滅 (100ms) | USB 未接続 |
| 低速点滅 (500ms) | USB 接続済み、ストリーミング停止中 |
| 常時点灯 | ストリーミング中 |

## ビルド方法

### 前提条件

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) がインストール済み
- `PICO_SDK_PATH` 環境変数が設定済み

### ビルド手順

```bash
# pico_sdk_import.cmake をコピー（初回のみ）
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .

# ビルドディレクトリ作成
mkdir build && cd build

# W5500-EVB-Pico2 (RP2350) の場合
cmake .. -DPICO_BOARD=pico2
make -j$(nproc)

# W5500-EVB-Pico (RP2040) の場合
# cmake .. -DPICO_BOARD=pico
# make -j$(nproc)
```

### 書き込み

1. W5500-EVB-Pico2 の BOOTSEL ボタンを押しながら USB 接続
2. マスストレージデバイスとして認識される
3. `build/w5500_livox_emulator.uf2` をドラッグ＆ドロップ

## 設定（ファームウェア書き換え不要）

ファームウェアは USB CDC 経由で ROS ノードから設定コマンドを受信する。
**一度書き込めば、ROS ノード側の launch パラメータを変えるだけで
異なる IP / シリアル番号の Livox として振る舞える。**

### 設定可能な項目

| 項目 | ROS パラメータ | デフォルト値 | 説明 |
|------|---------------|-------------|------|
| LiDAR IP | `lidar_ip` | 192.168.1.12 | エミュレートする LiDAR の IP |
| ホスト IP | `host_ip` | 192.168.1.5 | データ送信先（ロボット PC）の IP |
| シリアル番号 | `serial_number` | 0TFDFH600100511 | `livox_ros_driver2` に通知する SN |

設定は ROS ノード起動時に自動送信される。パラメータを変えて再起動すれば
別の LiDAR として動作する（ファームウェアの再書き込みは不要）。

### 複数台エミュレーション

W5500 の Source IP レジスタは 1 つのため、**1 ボード = 1 台の LiDAR** を
エミュレートする。複数台の Mid360 を同時エミュレートするには、台数分の
W5500-EVB-Pico2 が必要。

```
W5500-EVB-Pico2 #1 (192.168.1.12) ──> ロボット PC: LiDAR #1
W5500-EVB-Pico2 #2 (192.168.1.13) ──> ロボット PC: LiDAR #2
```

それぞれ ROS launch パラメータで `lidar_ip` と `serial_number` を変えるだけで良い。

## Livox SDK2 プロトコル実装

本ファームウェアが実装する Livox SDK2 チャネル:

| チャネル | LiDAR ポート | ホストポート | 説明 |
|---------|-------------|-------------|------|
| Discovery | 56000 | 56000 | ブロードキャスト検出応答 |
| Command | 56100 | 56101 | WorkMode 制御等 |
| PointCloud | 56300 | 56301 | 点群データ UDP 送信 |
| IMU | 56400 | 56401 | IMU データ UDP 送信 |

### 検出シーケンス

```
livox_ros_driver2           W5500-EVB-Pico2
       |                          |
       |-- LidarSearch (broadcast) -->|  UDP 56000
       |                          |
       |<-- DetectionData ACK ----|   dev_type=9 (Mid360), SN, IP
       |                          |
       |-- WorkModeControl(Normal) -->|  UDP 56100
       |                          |
       |<-- ACK ------------------|
       |                          |
       |<== PointCloud UDP ======|   UDP 56300 -> 56301
       |<== IMU UDP ============|   UDP 56400 -> 56401
```

## HILS フレームプロトコル（USB CDC）

PC ↔ ファームウェア間は共通の HILS フレームプロトコルを使用:

```
[0xAA][0x55][payload_length (4B LE)][payload (N bytes)][XOR checksum (1B)]
```

### メッセージタイプ

| タイプ | 方向 | 説明 |
|--------|------|------|
| 0x10 | PC → FW | 点群バッチ (data_type + dot_num + timestamp + points) |
| 0x11 | PC → FW | IMU データ (gyro_xyz + acc_xyz) |
| 0x12 | PC → FW | 設定コマンド (IP / SN / start / stop) |
| 0x18 | FW → PC | ステータス報告 (state + points_sent + udp_errors) |

## デバッグ

UART0 (GP0/GP1, 115200bps) にデバッグログが出力される:

```
=== W5500 Livox Mid360 Emulator ===
[W5500] SPI initialized, hardware reset complete
[W5500] Network: 192.168.1.12
[W5500] Socket 0 opened, port 56000
[W5500] Socket 1 opened, port 56100
[W5500] Socket 2 opened, port 56300
[W5500] Socket 3 opened, port 56400
[Livox] All sockets opened
[Main] Ready. Waiting for USB CDC connection...
[Main] LiDAR IP set to 192.168.1.12
[Main] Host IP set to 192.168.1.5
[Main] Serial number set
[Main] Emulation START
[Livox] Discovery from 192.168.1.5:56000
[Livox] WorkMode=1, streaming=1
```
