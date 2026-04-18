# hils_bridge_lidar_livox

Livox Mid360 HILS ブリッジ ROS 2 パッケージ。
シミュレーション (Isaac Sim / Gazebo) の PointCloud2 / IMU トピックを
W5500-EVB-Pico2 ファームウェアに USB CDC 経由で送信する。

## 概要

```
┌──────────────┐   PointCloud2    ┌───────────────────┐   USB CDC     ┌──────────────┐
│ Isaac Sim /  │  /livox/lidar    │ hils_bridge_      │  HILS frame   │ W5500-EVB-   │
│ Gazebo       │ ──────────────>  │ lidar_livox       │ ──────────>   │ Pico2        │
│              │  sensor_msgs/Imu │ (this package)    │               │ (firmware)   │
│              │  /livox/imu      │                   │               │              │
└──────────────┘ ──────────────>  └───────────────────┘               └──────────────┘
                                                                         │ Ethernet
                                                                         ▼
                                                                  ┌──────────────┐
                                                                  │ Robot PC     │
                                                                  │ livox_ros_   │
                                                                  │ driver2      │
                                                                  └──────────────┘
```

## 依存パッケージ

- `rclpy`, `sensor_msgs`, `std_msgs`
- `python3-numpy`, `python3-serial` (`pyserial`)
- `sensor_msgs_py` (PointCloud2 読み取り用)

## インストール

```bash
cd ~/colcon_ws
colcon build --packages-select hils_bridge_lidar_livox
source install/setup.bash
```

## 使い方

### 基本的な起動

```bash
# デフォルト設定で起動 (LiDAR IP: 192.168.1.12, ホスト IP: 192.168.1.5)
ros2 launch hils_bridge_lidar_livox livox_bridge.launch.py
```

### パラメータを指定して起動

```bash
ros2 launch hils_bridge_lidar_livox livox_bridge.launch.py \
  serial_port:=/dev/ttyACM1 \
  lidar_ip:=192.168.1.13 \
  host_ip:=192.168.1.100 \
  serial_number:=0TFDFH600200522 \
  max_hz:=10.0 \
  max_points_per_frame:=3000
```

### 複数台エミュレーション

それぞれ異なるパラメータで複数ノードを起動する:

```bash
# LiDAR #1
ros2 launch hils_bridge_lidar_livox livox_bridge.launch.py \
  serial_port:=/dev/ttyACM0 \
  lidar_ip:=192.168.1.12 \
  serial_number:=0TFDFH600100511

# LiDAR #2 (別ターミナル)
ros2 launch hils_bridge_lidar_livox livox_bridge.launch.py \
  serial_port:=/dev/ttyACM1 \
  lidar_ip:=192.168.1.13 \
  serial_number:=0TFDFH600200522
```

## パラメータ一覧

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|----------|------|
| `serial_port` | string | `/dev/ttyACM0` | W5500-EVB-Pico2 の USB CDC ポート |
| `pointcloud_topic` | string | `/livox/lidar` | 購読する PointCloud2 トピック |
| `imu_topic` | string | `/livox/imu` | 購読する IMU トピック |
| `enable_imu` | bool | `true` | IMU データ転送を有効化 |
| `max_hz` | float | `10.0` | 最大送信レート (Hz) |
| `max_points_per_frame` | int | `5000` | 1フレームあたり最大点数 |
| `point_data_type` | int | `1` | 1=CartesianHigh(14B/pt), 2=CartesianLow(8B/pt) |
| `lidar_ip` | string | `192.168.1.12` | エミュレートする LiDAR の IP |
| `host_ip` | string | `192.168.1.5` | データ送信先（ロボット PC）の IP |
| `serial_number` | string | `0TFDFH600100511` | エミュレートする LiDAR の SN |

### 動的パラメータ変更

一部のパラメータは実行中に変更可能:

```bash
# 1フレームあたりの点数を変更（USB帯域調整）
ros2 param set /hils_livox_bridge max_points_per_frame 3000

# 送信レートを変更
ros2 param set /hils_livox_bridge max_hz 20.0
```

## PointCloud2 の互換性

入力トピックには以下のフィールドを含む PointCloud2 を想定:

| フィールド | 型 | 必須 | 説明 |
|-----------|-----|------|------|
| `x` | float32 | 必須 | X座標 (メートル) → mm に変換 |
| `y` | float32 | 必須 | Y座標 (メートル) |
| `z` | float32 | 必須 | Z座標 (メートル) |
| `intensity` or `reflectivity` | float32/uint8 | 任意 | 反射強度 (0-255 にクランプ) |

Isaac Sim, Gazebo, 一般的な LiDAR シミュレーションプラグインの出力に対応。
`intensity` フィールドがない場合は `reflectivity=0` で送信される。

## USB 帯域の制限事項

USB Full Speed CDC の実効帯域は約 800 KB/s。

| point_data_type | 1点あたり | 5000点@10Hz | 帯域使用 |
|----------------|----------|-------------|---------|
| 1 (CartesianHigh) | 14 B | 700 KB/s | 87% |
| 2 (CartesianLow) | 8 B | 400 KB/s | 50% |

帯域不足の場合:
- `max_points_per_frame` を減らす（ダウンサンプリング）
- `point_data_type` を `2` にする（精度: mm → cm）
- `max_hz` を減らす

## ロボット PC 側の設定

`livox_ros_driver2` の設定例 (`MID360_config.json`):

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56201,
      "point_data_port": 56301,
      "imu_data_port": 56401,
      "log_data_port": 56501
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.5",
      "cmd_data_port": 56101,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "",
      "point_data_port": 56301,
      "imu_data_ip": "",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.12",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "x": 0, "y": 0, "z": 0
      }
    }
  ]
}
```

## トラブルシューティング

### シリアルポートが開けない

```bash
# デバイス確認
ls -la /dev/ttyACM*

# 権限付与
sudo chmod 666 /dev/ttyACM0
# または udev ルールを追加
```

### 点群が途切れる

USB 帯域不足の可能性。以下を試す:
```bash
ros2 param set /hils_livox_bridge max_points_per_frame 2000
ros2 param set /hils_livox_bridge point_data_type 2
```

### livox_ros_driver2 がデバイスを検出しない

1. ネットワーク設定を確認: ロボット PC の NIC が `192.168.1.0/24` 内にあること
2. ファームウェアの LED が低速点滅（USB接続済み）であること
3. `lidar_ip` / `host_ip` が `MID360_config.json` と一致していること
