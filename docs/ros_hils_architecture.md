# ROS Hardware-in-the-Loop Simulation (HILS) アーキテクチャ

## 1. 背景：ロボット開発におけるHILSの現状

### 1.1 ソフトウェアシミュレーションの成熟と残る課題

Gazebo、Unity、NVIDIA Isaac Simといったシミュレータは、ROSトピックを介して仮想センサデータを直接パブリッシュする形でのSoftware-in-the-Loop (SILS) を高い水準で実現している。特にNVIDIA Isaac Sim 5.0は2025年にオープンソース化され、物理エンジン・レンダリング・センサシミュレーションを統合した環境として注目されている。

しかし、これらのシミュレータには共通する構造的な限界がある：

| 課題 | 詳細 |
|------|------|
| **ドライバスタックの未検証** | Gazebo/Unity/Isaac SimはROSトピックに直接パブリッシュするため、実機で使うセンサドライバ（velodyne_driver, realsense2_cameraなど）が一切テストされない |
| **通信プロトコルの不在** | 実機では UDP、UART、I2C、CAN、USBなどの物理的な通信経路を経由するが、シミュレーションではこれらが完全にバイパスされる |
| **タイミング・リアルタイム制約** | ソフトウェアシミュレーションはハードリアルタイム制約を持たず、実機でのみ発生するタイミング起因のバグを検出できない |
| **電気的インテグレーション** | コネクタの接触不良、EMIノイズ、電圧レベル不整合などは物理層を経由しなければ発見できない |

### 1.2 既存のHILSアプローチ

#### ドローン分野（PX4 HITL）
PX4フライトスタックでは、GazeboがシリアルまたはUSB経由で実機のフライトコントローラ（Pixhawk等）にセンサデータを送信するHITL（Hardware-in-the-Loop Testing）が成熟している。フライトコントローラのファームウェアを実機と同一条件でテストできる点で、ロボット開発のHILSの先駆的事例といえる。

#### NVIDIA Isaac ROS + Jetson HIL
NVIDIAは、Isaac SimからJetson実機にカメラフィード・LiDARスキャンを送信し、Jetson上で実際のROS 2ノード（Isaac ROS）を動作させるHILワークフローを提供している。ただし、これはJetsonプラットフォームに限定される。

#### ros2_controlフレームワーク
ros2_controlの`hardware_interface`抽象化により、Mock/Fakeコンポーネントと実ハードウェアをプラグイン単位で切り替えられる。`topic_based_hardware_interfaces`を使えばROSトピック経由でハードウェアインターフェースを駆動できるが、これもROS内部で完結しており、物理通信経路のテストにはならない。

#### 商用HILプラットフォーム
dSPACE、NI VeriStand、Speedgoat、Typhoon HILなどの商用HILプラットフォームは主に自動車・航空宇宙分野で使われ、CAN、EtherCAT、アナログI/Oなどの物理インターフェースエミュレーションが可能だが、数百万〜数千万円規模のコストがかかる。

### 1.3 ギャップ：安価なHILSの不在

**ソフトウェアシミュレーション（SILS）と商用HILプラットフォームの間に大きなギャップが存在する。** ROSベースのロボットで、数千円〜数万円の範囲で物理通信経路を含むHILSを実現する手法は確立されていない。ここに本プロジェクトの意義がある。

---

## 2. 設計方針：インターフェース種別ごとに最適な方式を選択

### 2.1 方針の変遷

初期の設計では「全デバイスをRP2040経由のUSB接続で統一」する方針を検討していた。しかし、プロトタイプ検証を通じて、インターフェース種別ごとに最適な方式が異なることが判明した。

**現在の方針：マイコンが本当に必要な箇所にのみマイコンを使い、それ以外は既存のUSBデバイスやソフトウェアエミュレーションで代用する。**

| インターフェース | 当初の方式 | 現在の方式 | 変更理由 |
|----------------|-----------|-----------|---------|
| シリアルセンサ (UART) | RP2040 USB-CDC | FT234Xクロス接続 | マイコン不要。2個のFT234XをTX/RXクロスで接続し、両PCにシリアルポートとして認識させる |
| Ethernet接続デバイス (LiDAR等) | RP2040 + W5500 | USB-LANアダプタ + 純ソフトウェアエミュレーション | USB CDC帯域がボトルネック。USB-LANアダプタの方がシンプルかつ高性能 |
| USBカメラ (UVC) | RP2040 (TinyUSB UVC) | RP2040 (TinyUSB UVC) **変更なし** | UVCデバイスクラスのエミュレーションにはマイコンが必要 |

### 2.2 全体アーキテクチャ

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          シミュレーションPC                                    │
│                                                                              │
│  ┌──────────┐   ROSトピック    ┌──────────────────────────────────────┐      │
│  │ Gazebo / │ ──────────────> │ HILS Bridge Nodes                    │      │
│  │ Unity /  │   /scan          │                                      │      │
│  │ Isaac Sim│   /image_raw     │ ┌────────────────────────────────┐  │      │
│  │          │   /imu/data      │ │ livox_emulator_node            │  │      │
│  │          │   /gps/fix       │ │  PointCloud2 → Livox SDK2 UDP  │──┼──┐   │
│  │          │   /gps/vel       │ │  (USB-LANアダプタ経由で送信)      │  │  │   │
│  │          │                  │ ├────────────────────────────────┤  │  │   │
│  │          │                  │ │ uvc_bridge_node                │  │  │   │
│  │          │                  │ │  image_raw → JPEG → USB CDC    │──┼──┼─┐ │
│  │          │                  │ ├────────────────────────────────┤  │  │ │ │
│  │          │                  │ │ gps_bridge_node                │  │  │ │ │
│  │          │                  │ │  NavSatFix+TwistStamped         │  │  │ │ │
│  │          │                  │ │   → NMEA(GGA/RMC) → Serial      │──┼──┼─┼┐│
│  │          │                  │ ├────────────────────────────────┤  │  │ │││
│  │          │                  │ │ imu_bridge_node                │  │  │ │││
│  │          │                  │ │  Imu → WT901 binary → Serial   │──┼──┼─┼┼┤
│  └──────────┘                  │ └────────────────────────────────┘  │  │ ││││
│                                └──────────────────────────────────────┘  │ │││
└─────────────────────────────────────────────────────────────────────────┼─┼┼┼┘
                                                                         │ │││
                            USB-LANアダプタ (eth1等) ◄────────────────────┘ │││
                            USB micro-B (CDC) ◄────────────────────────────┘││
                            FT234X USB-Serial ◄─────────────────────────────┘│
                            FT234X USB-Serial ◄──────────────────────────────┘
                                         │ │ │ │
                                         ▼ ▼ ▼ ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                             実機PC (ROS)                                      │
│                                                                              │
│  ┌────────────────┐   ┌────────────────┐   ┌────────────────┐              │
│  │livox_ros_      │   │usb_cam /       │   │nmea_navsat_    │              │
│  │driver2         │   │cv_camera       │   │driver          │              │
│  │(UDP: eth1等)   │   │(/dev/video0)   │   │(/dev/ttyUSB0)  │              │
│  └───────┬────────┘   └───────┬────────┘   └───────┬────────┘              │
│          ▼                    ▼                     ▼                        │
│     /livox/lidar         /image_raw              /gps/fix                    │
│                                                                              │
│  ┌──────────────────────────────────────────────────────────────────┐        │
│  │ RP2040 (Pico#1 + Pico#2) は UVCカメラエミュレーションにのみ使用    │        │
│  │ LiDAR/シリアルセンサはマイコン不要                                  │        │
│  └──────────────────────────────────────────────────────────────────┘        │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. センサ・アクチュエータ別 HILS実現方式

### 3.1 シリアルセンサ（UART）— FT234Xクロス接続方式

**対象デバイス：** GPS受信機、一部のIMU、超音波センサ、シリアル接続のモータドライバ

#### 方式概要

FT234X超小型USBシリアル変換モジュール（秋月 [108461]）を2個使用し、TX/RXをクロス接続（ヌルモデム接続）する。各FT234XをそれぞれシミュレーションPCと実機PCにUSB接続すると、両PC上で `/dev/ttyUSB0` 等のシリアルポートとして認識される。

シミュレーションPC側のブリッジノードは自PC側のFT234Xのシリアルポートに対してNMEA等のプロトコルデータを書き込み、それがクロス接続を経由して実機PC側のFT234Xに到達する。RP2040等のマイコンは一切不要。

```
シミュレーションPC         FT234X #1      FT234X #2         実機PC

/gps/fix                                                  
    │                    ┌─────────┐    ┌─────────┐       
    ▼                    │ FT234X  │    │ FT234X  │       
HILS Bridge Node         │  TX ──────────── RX    │       
 - NavSatFix→NMEA変換    │  RX ──────────── TX    │       
 - シリアルポートに       │         │    │         │       
   書き込み              │         │    │         │       
    │                    └────┬────┘    └────┬────┘       
    └──USB──> /dev/ttyUSB0    │  UART        │   /dev/ttyUSB0 ──> nmea_navsat_driver
              (シミュPC側)     │  クロス接続   │   (実機PC側)        → /gps/fix
                              └──────────────┘
```

**利点：**
- マイコンのファームウェア開発が不要
- FT234Xモジュールは1個400円程度と安価（2個で800円）
- 実機と同じUSBシリアルデバイスとして認識されるため、ドライバとの互換性が高い
- 複数のシリアルセンサをエミュレーションする場合はFT234Xのペアを追加するだけ

**対応プロトコル例：**

| デバイス | プロトコル | ボーレート | 状態 |
|---------|-----------|-----------|------|
| GPS受信機 | NMEA 0183 (GGA/RMC) | 9600 bps | **実装済・動作確認済** (`hils_bridge_gps_nmea0183`) |
| Witmotion WT901 IMU | バイナリ (0x51/0x52/0x53/0x59) | 115200 bps | **実装済・動作確認済** (`hils_bridge_imu_witmotion_wt901`) |
| 超音波センサ | テキスト/バイナリ | 9600〜115200 bps | 計画 |

#### 検証済の組合せ

| エミュレータ | 実機ドライバ | 確認済トピック | 主な注意点 |
|-------------|------------|--------------|------------|
| `hils_bridge_gps_nmea0183` | `nmea_navsat_driver` | `/fix`, `/vel` | 上流 launch が `port:=/baud:=` 引数を無視するため `ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p ...` で起動 |
| `hils_bridge_imu_witmotion_wt901` | `witmotion_ros` (ElettraSciComp/witmotion_IMU_ros, ros2 ブランチ) | `/imu`, `/orientation`, `/magnetometer` | `libqt5serialport5-dev` 必須。Quaternion パケット (0x59) も送信するためデフォルト `use_native_orientation: true` でそのまま動作 |

### 3.2 Ethernet接続センサ（3D LiDAR等）— USB-LANアダプタ方式

**対象デバイス：** Livox Mid-360、Velodyne VLP-16、Ouster OS1、各種産業用カメラ

#### 方式概要

USB-LANアダプタをシミュレーションPCに接続し、増設したネットワークインターフェース（eth1等）にLiDARのIPアドレスを割り当てる。純ソフトウェアのエミュレータノードが、そのインターフェース経由でLiDAR互換のUDPパケットを送信する。

```
シミュレーションPC                                               実機PC

Gazebo/Unity/Isaac Sim
    │
    ▼ /points (PointCloud2)
┌──────────────────┐      USB-LANアダプタ (eth1)        ┌──────────────┐
│ livox_emulator_  │      IP: 192.168.1.12              │livox_ros_    │
│ node             │ ──────────────────────────────────> │driver2       │
│ - PointCloud2    │    Livox SDK2 UDPパケット            │              │
│   → SDK2変換     │    port 56301 (points)             │→ /livox/lidar│
│ - IMU→SDK2変換   │    port 56401 (IMU)                │→ /livox/imu  │
└──────────────────┘                                    └──────────────┘
```

#### Livox Mid-360 エミュレーション（実装済み）

| 項目 | 詳細 |
|------|------|
| プロトコル | Livox SDK2 (UDP) |
| ディスカバリ | UDP 56000 (broadcast) → 応答で接続確立 |
| コマンド | UDP 56100 (WorkModeControl等) |
| 点群データ | UDP 56300→56301 (CartesianHigh 14B / CartesianLow 8B) |
| IMUデータ | UDP 56400→56401 |
| 点群レート | 最大20,000点/フレーム @ 10Hz |

**帯域比較（RP2040+W5500方式 vs USB-LANアダプタ方式）：**

| 項目 | RP2040+W5500 (旧方式) | USB-LANアダプタ (現方式) |
|------|---------------------|----------------------|
| 実効帯域 | ~800 KB/s (USB CDC律速) | ~100 Mbps (Ethernet) |
| 最大点群数 | 3,000点@10Hz (CartesianHigh) | 20,000点@10Hz (制限なし) |
| ファームウェア | 必要 | 不要 |
| コスト | ~2,420円 (W5500-EVB-Pico2) | ~1,000円 (USB-LANアダプタ) |

USB-LANアダプタ方式が帯域・コスト・保守性すべてにおいて優れるため、こちらを推奨する。

#### 他のLiDARへの展開

同じUSB-LANアダプタ方式で、他のEthernet接続LiDARにも対応可能：

| LiDAR | プロトコル | データポート | 必要帯域 | 状態 |
|-------|-----------|------------|---------|------|
| Livox Mid-360 | Livox SDK2 (UDP) | 56301 | ~12 Mbps | **実装済み・動作確認済** |
| Velodyne VLP-16 | Velodyne UDP | 2368 | ~7.5 Mbps | **実装済み・動作確認済** |
| Ouster OS1 | Ouster UDP + HTTP API | 7502/80 | 10〜128 Mbps | **実装済み・動作確認済** |

#### ⚠️ 重要な注意点：実機ドライバはフォーマット違反を「警告なく」拒否する

HILSエミュレータ実装で最も注意すべき点として、**実機ドライバ（特に `velodyne_pointcloud`）はパケットフォーマットの違反に対して、ログ出力なしで該当ブロック全体を無効化する**傾向がある。

実際に Velodyne VLP-16 HILS 検証で遭遇した事例：

| 問題 | 症状 | 根本原因 |
|------|------|---------|
| バイト順誤り | `/velodyne_packets` は正常に配信されるが、`/velodyne_points` が常に空 (width=0) | ブロックフラグのバイト順を `0xEE 0xFF` で送信していたが、実機ドライバは `0xFF 0xEE` (uint16 LE = `0xEEFF`) を期待 |
| UDPペイロードサイズ誤り | ドライバがパケット受信できる場合とできない場合が混在 | Ethernetフレーム全長 (1248B) で送信していたが、UDPペイロードは1206B |

**エミュレータ実装時の鉄則：**

1. **公式仕様書のバイト並びを文字通りに実装する**。「flag=0xFFEE」と書かれていても、それがuint16値かバイト並びかを必ず確認する
2. **UDPペイロードサイズはEthernetフレームサイズと区別する**。1248バイト vs 1206バイト のような数字の食い違いに注意
3. **パケット生成後は `tcpdump -X` で16進ダンプを取り、公式ドキュメントのバイト並びと完全一致することを確認する**
4. **ドライバ側のトピック（例: `/velodyne_packets`）が流れているだけでは成功とは言えない**。必ず最終的な変換結果（`/velodyne_points` の width/height）まで見る

#### 複数LiDAR構成

複数のLiDARをエミュレーションする場合は、USB-LANアダプタを台数分用意する：

```
シミュレーションPC
    │
    ├── USB-LANアダプタ #1 (eth1: 192.168.1.12) → Livox Mid-360 #1 エミュレーション
    ├── USB-LANアダプタ #2 (eth2: 192.168.2.12) → Livox Mid-360 #2 エミュレーション
    └── USB-LANアダプタ #3 (eth3: 192.168.3.12) → Velodyne VLP-16 エミュレーション
         │          │          │
         ▼          ▼          ▼
      実機PC (各インターフェースにLANケーブルで接続)
```

### 3.3 USBカメラ（UVC）— RP2040方式 **※マイコンが必要な唯一の用途**

**対象デバイス：** Webカメラ、RealSense（部分的）、各種USBカメラ

#### 方式概要

UVC（USB Video Class）デバイスとしてPCに認識させるためには、USBデバイスとして振る舞うマイコンが必要である。これはFT234Xやソフトウェアエミュレーションでは実現できないため、RP2040（TinyUSB）を使用する唯一のケースとなる。

**アーキテクチャ：** Pico 2台構成

```
シミュレーションPC          Pico#1                  Pico#2               実機PC
                       (CDC-SPI sender)        (UVC bridge)

/image_raw                                                          
    │                                                               
    ▼                                                               
uvc_bridge_node                                                     
 - OpenCV JPEG圧縮         SPI master              SPI slave         
 - USB CDC送信 ──USB──> Pico#1 ──UART 4Mbps──> Pico#2              
                       (データ中継)             TinyUSB UVC          
                                               Isochronous転送      
                                               wMaxPacketSize=256   
                                               ──USB──> /dev/video0
                                                         → usb_cam
                                                         → /image_raw
```

**実績（動作確認済み）：**

| 解像度 | 圧縮方式 | フレームレート | 備考 |
|--------|---------|--------------|------|
| 320x240 | MJPEG | 安定動作 | テスト用途 |
| 640x480 | MJPEG | ~17 fps | 多くのロボット用途に実用的 |
| 1280x720 | MJPEG | 低fps | 帯域制約あり（wMaxPacketSize=256） |

**設計上の重要な判断：**
- **Isochronous転送を採用**：Bulk転送はTinyUSBの`ep_status.busy`がストリーム停止時にクリアされず、再接続不可だったため断念
- **wMaxPacketSize=256**：Raspberry Pi 4でキーボードと共存するため、USBバス帯域予約を17%に制限
- **解像度切替**：逆方向通信（実機PC→Pico#2→Pico#1→シミュPC）で解像度コマンドを伝搬し、動的に変更可能
- **JPEG品質**：`ros2 param set`で動的変更可能

### 3.4 I2C/SPIセンサ

**対象デバイス：** MPU-6050/9250, BMI088, BMP280, ADXLシリーズ, 各種環境センサ

マイコンをI2C/SPIスレーブとして動作させ、対象センサのレジスタマップを模擬する。RP2040のPIO（Programmable I/O）が特に有効。

```
シミュレーションPC --> USB --> [RP2040] --I2C/SPI (スレーブ)--> [実機PCのI2Cマスタ / マイコン]
```

**課題：**
- センサごとにレジスタマップが異なるため、対象センサの仕様書に基づいた個別実装が必要
- WHO_AM_I レジスタ等の識別レジスタも正確に応答する必要がある（例: MPU-6050 → 0x68）

### 3.5 CANバスデバイス

**対象デバイス：** モータドライバ（CANopen）、車両ECU、ロボットアクチュエータ

マイコン + CANトランシーバで実現。ESP32-S3はCAN互換のTWAIペリフェラルを内蔵しており、外付けCANコントローラ不要。

```
シミュレーションPC --> USB --> [ESP32-S3 + CANトランシーバ] --CAN bus--> [実機PC + CAN I/F]
```

### 3.6 PWM/エンコーダ（モータ制御系）

**対象デバイス：** RCサーボ、BLDCモータ（ESC）、ステッピングモータ、ロータリエンコーダ

RP2040のPIOによりエンコーダパルス（A/B相）を高精度に生成可能。

---

## 4. ROSトピック→HILS変換の設計パターン

### 4.1 HILS Bridge Nodeの設計

各センサタイプに対応するROSノードを実装し、ROSトピックをプロトコル変換して物理インターフェースに出力する。

```python
# 設計パターンの概念コード (Python / rclpy)

class HilsBridgeNode(Node):
    """ROSトピック → 物理インターフェース変換の基本パターン"""
    
    def __init__(self):
        super().__init__('hils_bridge')
        
        # LiDAR: PointCloud2 → Livox SDK2 UDPパケット (USB-LANアダプタ経由)
        self.create_subscription(PointCloud2, '/livox/lidar', self.lidar_callback, 10)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # network_interfaceパラメータでバインドするインターフェースを指定可能
        
        # GPS: NavSatFix → NMEA over Serial (FT234Xクロス接続経由)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
        
        # Camera: Image → JPEG → USB CDC (RP2040経由)
        self.create_subscription(Image, '/image_raw', self.camera_callback, 10)
        self.cdc_port = serial.Serial('/dev/ttyACM0', 115200)
        
    def lidar_callback(self, msg):
        packets = self.pointcloud2_to_livox_sdk2_packets(msg)
        for pkt in packets:
            self.udp_socket.sendto(pkt, (LIDAR_IP, 56301))
    
    def gps_callback(self, msg):
        nmea = self.navsatfix_to_nmea(msg)
        self.serial_port.write(nmea.encode())
    
    def camera_callback(self, msg):
        jpeg = self.image_to_jpeg(msg)
        self.cdc_port.write(self.frame_protocol_encode(jpeg))
```

### 4.2 テスト戦略

HILSの導入により、以下の段階的テストが可能になる：

```
Level 0: 純粋シミュレーション (SILS)
  Gazebo/Unity/Isaac Sim → ROSトピック → アプリケーション
  ※ドライバ・物理層は未テスト

Level 1: ドライバレベルHILS
  シミュレーション → プロトコル変換 → 実機ドライバ → ROSトピック → アプリケーション
  ※物理通信経路はソフトウェアエミュレーション（v4l2loopback, localhost UDP等）

Level 2: 物理層HILS
  シミュレーション → 物理ケーブル → 実機PC → ドライバ → ROSトピック → アプリ
  ※USB, UART, Ethernet等の物理経路を含む

Level 3: 完全HILS
  シミュレーション → 実機と同一のセンサ/アクチュエータI/F
  ※電源、EMI、コネクタを含む実環境に近い条件
```

---

## 5. 推奨ハードウェア構成

### 5.1 方式別ハードウェア一覧

```
┌─────────────────────────────────────────────────────────────┐
│              推奨HILSハードウェア構成（改訂版）                 │
├──────────────────┬──────────────────────────────────────────┤
│ シリアルセンサ     │ FT234X × 2 (TX/RXクロス接続)              │
│ (GPS, IMU, 超音波) │ → 実機PCでは /dev/ttyUSB0               │
│                  │ → マイコン不要                            │
├──────────────────┼──────────────────────────────────────────┤
│ 3D LiDAR         │ USB-LANアダプタ + 純ソフトウェアエミュレータ │
│ (Livox, Velodyne) │ → USB-LANにLiDAR IPを割当               │
│                  │ → マイコン不要                            │
├──────────────────┼──────────────────────────────────────────┤
│ カメラ            │ RP2040 × 2 (Pico#1 + Pico#2)            │
│                  │ → TinyUSB UVC + MJPEG                    │
│                  │ → マイコンが必要な唯一の用途               │
├──────────────────┼──────────────────────────────────────────┤
│ I2C/SPIセンサ     │ RP2040 × 1 (PIO スレーブ)               │
│ (加速度計, ジャイロ)│ → 必要な場合のみ                         │
├──────────────────┼──────────────────────────────────────────┤
│ CANデバイス       │ ESP32-S3 + CANトランシーバ               │
│ (モータドライバ)   │ → 必要な場合のみ                         │
├──────────────────┼──────────────────────────────────────────┤
│ エンコーダ/PWM    │ RP2040 (PIO)                            │
│                  │ → 必要な場合のみ                         │
└──────────────────┴──────────────────────────────────────────┘
```

### 5.2 最小構成

多くのロボットで使われるLiDAR + カメラの構成を最小構成とする：

| 用途 | ハードウェア | 概算コスト |
|------|------------|----------|
| LiDAR (Livox等) | USB-LANアダプタ × 1 | ~1,000円 |
| カメラ (UVC) | Raspberry Pi Pico H × 2 + USBケーブル | ~2,200円 |
| 配線・ジャンパ | ブレッドボード、ジャンパワイヤ | ~500円 |
| **合計** | | **~3,700円** |

シリアルセンサを追加する場合は FT234Xのペア（~800円/組）を追加するだけ。

---

## 6. 実現可能性のまとめ

### 6.1 各インターフェースの実現可能性一覧

| センサ/デバイス | インターフェース | エミュレーション方式 | 実現難易度 | 推奨HW | 状態 |
|---------------|----------------|-------------------|----------|--------|------|
| GPS受信機 | UART (NMEA 0183) | FT234Xクロス接続 + NMEA(GGA/RMC)生成 | ★☆☆☆☆ | FT234X | **実装済・動作確認済** |
| シリアルIMU (Witmotion WT901) | UART (バイナリ) | FT234Xクロス接続 + WT901バイナリ生成 | ★★☆☆☆ | FT234X | **実装済・動作確認済** |
| 3D LiDAR (Livox Mid-360) | Ethernet UDP | USB-LANアダプタ + 純ソフトウェア | ★★☆☆☆ | USB-LANアダプタ | **実装済・動作確認済** |
| 3D LiDAR (Velodyne VLP-16) | Ethernet UDP | USB-LANアダプタ + 純ソフトウェア | ★★☆☆☆ | USB-LANアダプタ | **実装済・動作確認済** |
| 3D LiDAR (Ouster OS1) | Ethernet UDP + HTTP API | USB-LANアダプタ + 純ソフトウェア | ★★★☆☆ | USB-LANアダプタ | **実装済・動作確認済** |
| USBカメラ (MJPEG) | USB UVC | RP2040 × 2 + TinyUSB UVC | ★★★☆☆ | RP2040 (Pico H) | **実装済み** |
| I2C IMU (MPU6050等) | I2C スレーブ | RP2040 PIO I2Cスレーブ | ★★★☆☆ | RP2040 (PIO) | 未実装 |
| CANモータドライバ | CAN bus | ESP32-S3 TWAI + CANトランシーバ | ★★★☆☆ | ESP32-S3 | 未実装 |
| RCサーボ | PWM | マイコン PWM出力/入力 | ★☆☆☆☆ | 汎用マイコン | 未実装 |
| エンコーダ | パルス (A/B相) | RP2040 PIO | ★★☆☆☆ | RP2040 (PIO) | 未実装 |

### 6.2 結論

1. **インターフェース種別ごとに最適な方式を選択することで、マイコンの使用を最小限に抑えつつ、安価なHILSが実現可能。** マイコン（RP2040）が必要なのはUVCカメラエミュレーションのみであり、シリアルセンサはFT234Xクロス接続、LiDARはUSB-LANアダプタ＋純ソフトウェアで対応できる

2. **3D LiDAR 3機種 (Livox Mid-360 / Velodyne VLP-16 / Ouster OS1) のHILSが実装・動作確認済み。** いずれも純Pythonで対応プロトコル (Livox SDK2 / Velodyne UDP / Ouster UDP+HTTP) を実装。USB-LANアダプタにバインドして実機ドライバ (`livox_ros_driver2`, `velodyne_driver`, `ouster_ros`) で受信できることを確認

3. **シリアルセンサ (GPS NMEA / Witmotion WT901 IMU) のHILSも実装・動作確認済み。** FT234X 2 個のクロス接続で `nmea_navsat_driver`, `witmotion_ros` (ElettraSciComp) と接続。WT901 は標準 4 種パケット (0x51/0x52/0x53/0x59) を全て出力し実機互換

4. **UVCカメラのHILSも実装・動作確認済み。** RP2040 2台構成でMJPEGパススルー。640x480@17fps、Raspberry Pi 4でキーボード共存可能

5. **最小構成（LiDAR + カメラ）は約3,700円で構築可能。** 従来の見積もり（~17,500円）から大幅にコストダウン

6. **ros2_controlの`hardware_interface`抽象化を活用**することで、HILSと実機の切替をソフトウェア設定のみで実現できる設計が望ましい

---

## 参考資料

- [NVIDIA Hardware-in-the-Loop with Jetson](https://developer.nvidia.com/blog/design-your-robot-on-hardware-in-the-loop-with-nvidia-jetson/)
- [PX4 HITL Simulation](https://docs.px4.io/main/en/simulation/hitl.html)
- [ros2_control Mock Components](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)
- [TinyUSB - Open Source USB Stack](https://github.com/hathach/tinyusb)
- [Livox SDK Communication Protocol](https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol)
- [v4l2loopback](https://github.com/umlaeute/v4l2loopback)
