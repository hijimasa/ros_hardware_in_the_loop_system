# ROS Hardware-in-the-Loop Simulation (HILS) アーキテクチャ検討

## 1. 背景：ロボット開発におけるHILSの現状

### 1.1 ソフトウェアシミュレーションの成熟と残る課題

Gazebo、Unity、NVIDIA Isaac Simといったシミュレータは、ROSトピックを介して仮想センサデータを直接パブリッシュする形でのSoftware-in-the-Loop (SILS) を高い水準で実現している。特にNVIDIA Isaac Sim 5.0は2025年にオープンソース化され、物理エンジン・レンダリング・センサシミュレーションを統合した環境として注目されている。

しかし、これらのシミュレータには共通する構造的な限界がある：

| 課題 | 詳細 |
|------|------|
| **ドライバスタックの未検証** | Gazebo/Isaac SimはROSトピックに直接パブリッシュするため、実機で使うセンサドライバ（velodyne_driver, realsense2_cameraなど）が一切テストされない |
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

## 2. UIAPduinoの概要と適用可能性

### 2.1 UIAPduinoとは

UIAPduino は日本のUIAP社が開発するArduino互換の開発ボードシリーズで、WCH製RISC-Vマイコンを搭載し、SparkFun Pro Micro互換のフォームファクタを持つ。

**UIAPduino Pro Micro CH32V003 V1.4（主力製品）の主要スペック：**

| 項目 | 仕様 |
|------|------|
| MCU | CH32V003F4U6 (32bit RISC-V, 48MHz) |
| SRAM | 2 KB |
| Flash | 16 KB |
| GPIO | 最大18ピン |
| ADC | 8チャンネル, 10bit |
| USB | USB 2.0 Low-Speed (Type-C, デバイスモード, HID対応) |
| 通信 | UART, I2C, SPI |
| 電圧 | 3.3V / 5V 切替可能 |
| 対応IDE | Arduino IDE 2.3.2+ |
| 価格帯 | 非常に安価 |

**ラインナップ：**
- UIAPduino Pro Micro CH32V003 V1.4（GA）
- UIAPduino Pro Micro CH32V006 V1.0/V1.1（Beta）
- UIAPduino Pro Micro CH32V002（準備中）

### 2.2 UIAPduinoの強みと制約

**強み：**
- 非常に安価で入手しやすい
- Arduino IDE対応で開発が容易
- UART, I2C, SPI対応により基本的なセンサエミュレーションが可能
- 小型フォームファクタ

**制約：**
- USB Low-Speed（1.5 Mbps）のためUSBデバイスエミュレーション能力が限定的（HIDのみ、CDC/UVC不可）
- 2KB SRAM / 16KB Flashと非常にリソースが少ない
- Ethernet非対応
- CAN非対応

### 2.3 UIAPduino 2台構成でのシリアルHILS

以前、USBシリアル変換器2つによるループバック構成でシリアルデバイスのHILSが実現可能なことが確認されている。UIAPduinoでも同様のアプローチが検討できるが、**注意点がある**：

```
[シミュレーションPC] --USB--> [UIAPduino #1 (UART TX)] --ワイヤ--> [UIAPduino #2 (UART RX)] --USB--> [実機PC]
```

- UIAPduino CH32V003のUSBはLow-Speed (HID)のみで、CDCクラス（仮想シリアルポート）をネイティブにサポートしない
- UARTピンを直結してデータを中継する構成は可能だが、USB-シリアル変換器として認識させるにはCDC対応が必要
- **代替案**: UIAPduinoのUARTピンと別途のUSB-シリアル変換チップ（FT232R, CP2102, CH340等）を組み合わせる

**より適切な選択肢としては、RP2040（Raspberry Pi Pico）やESP32-S2/S3**が挙げられる。これらはUSB Full-Speed（12 Mbps）でCDCクラスをネイティブサポートし、TinyUSBライブラリによりUSBデバイスとして柔軟に動作する。価格も数百円〜千円程度で十分安価である。

---

## 3. センサ・アクチュエータ別 HILS実現方式

### 3.1 設計方針：全デバイスをUSBハブ経由のRP2040で統一

本プロジェクトでは、**すべてのセンサ・アクチュエータエミュレーションをRP2040経由のUSB接続で統一する**方針をとる。

**理由：**
- 3D LiDARを複数使用する場合、LAN直結方式ではLANポートが台数分必要になるが、通常のPCはLANポートを1つしか持たない
- USBハブ経由であれば、エミュレーション対象デバイスが増えてもRP2040を追加接続するだけでスケール可能
- 全デバイスのインターフェースが統一されることで、HILS環境の構築・管理が容易になる
- カメラについても、GStreamer等でLAN経由にすると物理層がEthernetに変わってしまい、実機のUSBカメラとは異なる通信経路になるためHILSの意義が薄れる

### 3.2 全体アーキテクチャ

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        シミュレーションPC                                  │
│                                                                         │
│  ┌──────────┐   ROSトピック    ┌──────────────────────────────────┐     │
│  │ Gazebo / │ ──────────────> │ HILS Bridge Node                 │     │
│  │ Isaac Sim│   /scan          │ (トピック→デバイスプロトコル変換)   │     │
│  │          │   /image_raw     │                                  │     │
│  │          │   /imu/data      │  各RP2040へUSB経由でデータ送信    │     │
│  │          │   /gps/fix       │                                  │     │
│  │          │   /cmd_vel       └───────┬──────────────────────────┘     │
│  │          │   /joint_states          │ USB (CDC/Vendor)               │
│  └──────────┘                          │                                │
└────────────────────────────────────────┼────────────────────────────────┘
                                         │
                                    USB ケーブル
                                         │
                                         ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                         RP2040 HILSデバイス群                             │
│                                                                          │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐      │
│  │ RP2040 #1        │  │ RP2040 #2        │  │ RP2040 #3        │      │
│  │ [LiDAR Emu]      │  │ [Camera Emu]     │  │ [GPS/IMU Emu]    │      │
│  │ USB CDC受信       │  │ USB CDC受信       │  │ USB CDC受信       │      │
│  │ → W5500          │  │ → TinyUSB UVC    │  │ → TinyUSB CDC    │      │
│  │ → UDP送信        │  │ → USBカメラ出力   │  │ → UART/シリアル   │      │
│  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘      │
│           │ Ethernet            │ USB                   │ USB            │
└───────────┼─────────────────────┼───────────────────────┼────────────────┘
            │                     │                       │
            ▼                     ▼                       ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                           実機PC (ROS)                                    │
│                                                                          │
│  ┌───────────────┐    ┌──────────────┐    ┌───────────────┐             │
│  │velodyne_driver│    │usb_cam /     │    │nmea_navsat_   │             │
│  │(UDP: eth1等)  │    │cv_camera     │    │driver         │             │
│  └───────┬───────┘    └──────┬───────┘    └───────┬───────┘             │
│          ▼                   ▼                    ▼                      │
│       /scan              /image_raw           /gps/fix                   │
│                                                                          │
│  ※ USBハブで接続。デバイス追加時はRP2040を増設するだけ                       │
└──────────────────────────────────────────────────────────────────────────┘
```

**スケーラビリティの例：LiDAR 3台構成**

```
                    USBハブ (実機PC側)
                    ┌─────┴─────┐
                    │           │
          ┌────────┴──┐  ┌────┴────────┐
          │ USB Eth #1│  │ USB Eth #2  │  ...
          │ (eth1)    │  │ (eth2)      │
          └─────┬─────┘  └──────┬──────┘
                │               │
  RP2040+W5500 #1    RP2040+W5500 #2    RP2040+W5500 #3
  (Front LiDAR)      (Left LiDAR)       (Right LiDAR)
                    ┌─────┬─────┐
                    │ USBハブ    │
                    │(シミュPC側)│
                    └─────┴─────┘

※ 通常のPCはLANポート1つだが、USBハブ経由なら容易に拡張可能
```

### 3.2 シリアルセンサ（UART/USB-CDC）

**対象デバイス：** GPS受信機、一部のIMU、超音波センサ、シリアル接続のモータドライバ

#### GPS受信機エミュレーション（NMEA over UART）

| 項目 | 詳細 |
|------|------|
| プロトコル | NMEA 0183 (テキストベース) |
| ボーレート | 9600 または 115200 bps |
| 主要センテンス | $GPGGA (測位), $GPRMC (推奨最小), $GPVTG (速度) |
| エミュレーション難易度 | **非常に容易** |

**実現方法：**
1. シミュレーションのROSトピック `/gps/fix` (sensor_msgs/NavSatFix) をサブスクライブ
2. 緯度・経度・高度をNMEA形式文字列に変換
3. マイコン (RP2040等) のUSB-CDC経由で実機PCのシリアルポートとして出力
4. 実機PCの `nmea_navsat_driver` が通常通りパースしてROSトピックを発行

```
シミュレーションPC                      実機PC
/gps/fix --> NMEA変換 --> USB --> RP2040 (CDC) --> /dev/ttyACM0 --> nmea_navsat_driver --> /gps/fix
```

**既存ツール：** gpsfeed+, nmea-gps-emulator (Python), GPSSimulator（フォールトインジェクション対応）

#### IMUエミュレーション（UART経由）

多くのIMU（Xsens MTi, MicroStrain, Witmotion等）はUART経由でバイナリまたはASCIIプロトコルで通信する。対象IMUのプロトコルに準拠したデータをUSB-CDC経由で送信すれば、実機のIMUドライバがそのまま動作する。

#### UIAPduino 2台構成の評価

| 構成 | 実現可否 | 備考 |
|------|---------|------|
| UIAPduino 2台 (USB HID + UART中継) | △ | CDCクラス非対応のため、実機PCからシリアルポートとして認識されない。USB HIDを経由した独自プロトコルなら可能だが、既存のROSドライバとの互換性がない |
| UIAPduino + USB-シリアル変換チップ | ○ | UIAPduinoのUART出力をFT232R等に接続すれば、実機PCからは通常のシリアルデバイスとして認識される |
| RP2040 (Raspberry Pi Pico) 単体 | ◎ | TinyUSB CDCにより直接仮想シリアルポートとして認識。最も簡潔 |

### 3.3 I2C/SPIセンサ

**対象デバイス：** MPU-6050/9250, BMI088, BMP280, ADXLシリーズ, 各種環境センサ

#### エミュレーション方式

マイコンをI2C/SPIスレーブとして動作させ、対象センサのレジスタマップを模擬する。

```
シミュレーションPC --> USB --> [RP2040] --I2C/SPI (スレーブ)--> [実機PCのI2Cマスタ / マイコン]
```

**RP2040のPIO (Programmable I/O) が特に有効：**
- PIOステートマシンにより、I2C/SPIスレーブプロトコルをハードウェアレベルで実装可能
- CPUに依存しない高精度なタイミング制御
- 複数のプロトコルを同時にエミュレート可能

**課題：**
- センサごとにレジスタマップが異なるため、対象センサの仕様書に基づいた個別実装が必要
- WHO_AM_I レジスタ等の識別レジスタも正確に応答する必要がある（例: MPU-6050 → 0x68）
- I2C/SPIのスレーブモードは多くのマイコンで対応が限定的

### 3.4 CANバスデバイス

**対象デバイス：** モータドライバ（CANopen）、車両ECU、ロボットアクチュエータ

| 項目 | 詳細 |
|------|------|
| プロトコル | CAN 2.0A/B, CAN FD, CANopen (CiA 301/402) |
| 物理層 | 差動2線 (CAN_H / CAN_L) |
| 速度 | 125kbps 〜 1Mbps (CAN FD: 最大8Mbps) |
| エミュレーション難易度 | 中程度 |

**実現方法：**
- マイコン + CANトランシーバで実現（例: ESP32内蔵TWAI + トランシーバ, Arduino + MCP2515/MCP2518FD）
- ROSトピックの `/joint_states` や `/cmd_vel` をCANフレームに変換
- エンコーダフィードバック、ステータスフレームを生成して応答

```
シミュレーションPC --> USB --> [ESP32 + CANトランシーバ] --CAN bus--> [実機PC + CAN I/F]
```

### 3.5 Ethernet接続センサ（3D LiDAR等）

**対象デバイス：** Velodyne VLP-16, Ouster OS1, Livox Mid-360, 各種産業用カメラ

#### 3D LiDARエミュレーション

3D LiDARの多くはEthernet上のUDPパケットで点群データを送信する。プロトコルが公開されているため、ソフトウェアでのエミュレーションが比較的容易。

**Velodyne VLP-16の通信仕様：**

| 項目 | 詳細 |
|------|------|
| データポート | UDP 2368 |
| ポジションポート | UDP 8308 |
| パケットサイズ | 1248 bytes/パケット |
| パケット構造 | プロトコルヘッダ(42B) + データブロック×12 + タイムスタンプ(4B) + ファクトリ(2B) |
| パケットレート | 約754パケット/秒 |
| ポイントレート | 約300,000点/秒 |
| リターンモード | Strongest (0x37), Last (0x38), Dual (0x39) |

**Ouster OS1の通信仕様：**

| 項目 | 詳細 |
|------|------|
| LiDARデータポート | UDP 7502 |
| IMUデータポート | UDP 7503 |
| LiDARパケットレート | 640 Hz |
| IMUパケットレート | 100 Hz |
| エンディアン | リトルエンディアン |

**Livox Mid-360の通信仕様：**

| 項目 | 詳細 |
|------|------|
| プロトコル | UDP (Livox SDK2) |
| 同期 | IEEE 1588v2.0 PTP対応 |
| エンディアン | リトルエンディアン |

**エミュレーション方式：**

```
シミュレーションPC                                              実機PC
                                                              
Gazebo/Isaac Sim                                              
    │                                                         
    ▼ /points (PointCloud2)                                   
┌──────────────┐       Ethernet (UDP)        ┌──────────────┐
│ HILS Bridge  │ ──────────────────────────> │velodyne_     │
│ - PointCloud2│    VLP-16パケット形式         │driver        │
│   → Velodyne │    port 2368               │              │
│   パケット変換 │                             │→ /scan       │
└──────────────┘                             │→ /points     │
                                             └──────────────┘
```

**実現方法：RP2040 + W5500 によるEthernet出力**

本プロジェクトでは、LiDARエミュレーションもRP2040経由で統一する。シミュレーションPCからUSB CDC経由で点群データを受信し、RP2040がW5500を通じてVelodyne/Ouster互換のUDPパケットを送信する。

```
シミュレーションPC                 RP2040 + W5500              実機PC
                                                             
/points (PointCloud2)             USB CDC で受信              
    │                                 │                      
    ▼                                 ▼                      
HILS Bridge ──USB CDC──> RP2040: パケット構築     Ethernet
(バイナリ点群データ送信)        → W5500 UDP送信 ────────> velodyne_driver
                              port 2368                (UDP受信)
```

**帯域見積もり：**

| LiDAR | 必要帯域 | W5500実効帯域 (SPI 33MHz) | マージン | 対応可否 |
|-------|---------|-------------------------|---------|---------|
| Velodyne VLP-16 | ~7.5 Mbps (754pkt/s × 1248B) | ~20 Mbps | 2.6x | ○ 十分 |
| Ouster OS1-16/32 | ~10-25 Mbps | ~20 Mbps | 0.8-2.0x | ○〜△ チャンネル数依存 |
| Ouster OS1-128 | ~128 Mbps | ~20 Mbps | 0.16x | × 帯域不足 |
| Livox Mid-360 | ~12 Mbps | ~20 Mbps | 1.7x | ○ |

**注意：** Ouster OS1-128等の超高チャンネルLiDARはW5500の帯域では不足する。この場合はRP2040のPIOでSPIクロックを高速化（最大62.5MHz）するか、W5500の代わりにENC28J60（10Mbps）よりも高速なW6100（最大100Mbps実効）を検討する。ただし多くのロボット用途ではVLP-16/OS1-32クラスが一般的であり、W5500で十分対応可能。

**複数LiDAR構成の利点：** LAN直結方式ではLiDAR台数分のLANポートが必要だが、RP2040+W5500方式ならUSBハブに接続するRP2040を増やすだけでスケールできる。実機PC側にはRP2040のW5500から来るEthernet接続が必要だが、これもUSBイーサネットアダプタで増設可能。

#### GigE Visionカメラエミュレーション

産業用カメラで使われるGigE Visionプロトコルについても、ソフトウェアエミュレーションが可能：

- **GigESim** (A&B Software)：PCをGigE Visionカメラとして動作させる商用SDK
- **Basler pylon**：カメラエミュレーショントランスポートレイヤを内蔵
- **オープンソース**: GigE-Cam-Simulator (GitHub)

### 3.6 USBカメラ（UVC）

**対象デバイス：** Webカメラ、RealSense（部分的）、各種USBカメラ

#### マイコンによるUVCエミュレーション

TinyUSBライブラリはUVC（USB Video Class）デバイスクラスをサポートしており、RP2040でのUVCカメラエミュレーションが実証されている。

| 方式 | 解像度 | フレームレート | 備考 |
|------|--------|--------------|------|
| RP2040 + TinyUSB UVC (YUYV) | 320x240 | ~6.8 fps | 非圧縮、帯域制約大 |
| RP2040 + TinyUSB UVC (MJPEG) | 640x480〜1280x720 | 5〜17 fps | **シミュPC側でJPEGエンコード、RP2040はパススルー** |
| STM32H7 / Teensy 4.x (MJPEG) | 1920x1080 | ~30 fps | USB High-Speed (480Mbps) 対応 |

#### RP2040 UVC + MJPEG による高解像度対応（推奨方式）

USB Full-Speed (12Mbps) の帯域制約下でも、**MJPEG圧縮を活用すればfpsを落とすことで十分な解像度が確保できる**。実際の市販USBカメラでも同様のトレードオフ（高解像度では低fps）は一般的に行われている。

**ポイント：RP2040自体はJPEGエンコードしない。** シミュレーションPC側のHILS Bridge NodeがOpenCV等でJPEGエンコードした圧縮フレームをUSB CDC経由でRP2040に送信し、RP2040はそれをUVC MJPEGフレームとしてそのまま出力する。

```
シミュレーションPC                    RP2040                    実機PC
                                                              
Gazebo → /image_raw                 USB CDC で受信            
    │                                   │                     
    ▼                                   ▼                     
HILS Bridge Node               TinyUSB UVC デバイス           
 - OpenCV JPEG圧縮              - MJPEGフレームとして出力       
 - USB CDC送信 ──────────>      - /dev/video0 として認識 ──> usb_cam
                                                              → /image_raw
```

**USB Full-Speed (12Mbps) での解像度・fps見積もり：**

Isochronous転送の実効帯域を ~1025 KB/s として算出：

| 解像度 | MJPEG圧縮率 | フレームサイズ | 達成fps | 備考 |
|--------|-----------|-------------|--------|------|
| 640x480 | 10:1（標準） | ~60 KB | **~17 fps** | 十分実用的 |
| 640x480 | 20:1（高圧縮） | ~30 KB | **~34 fps** | Webカメラ相当 |
| 1280x720 | 10:1 | ~180 KB | **~5.7 fps** | 低fpsだが動作確認には十分 |
| 1280x720 | 20:1 | ~90 KB | **~11 fps** | ロボット用途で実用的 |
| 1920x1080 | 20:1 | ~202 KB | **~5 fps** | 最低限の動作確認向け |

**640x480 / 10〜17 fps は多くのロボットカメラ用途で実用的な値**であり、ナビゲーション用のカメラや物体検出の基本テストには十分対応できる。高fpsが必要な場合はUSB High-Speed対応マイコン（STM32H7, Teensy 4.x）に置き換えることで対応可能。

**実装上の注意：**
- RP2040は2つのUSBエンドポイントを同時に使用する：CDC（シミュPCからのデータ受信）とUVC（実機PCへの映像出力）
- TinyUSBはComposite Device（複数USBクラスの同時使用）をサポートしている
- RP2040の264KB SRAMは数フレーム分のMJPEGバッファとして十分

### 3.7 PWM/エンコーダ（モータ制御系）

**対象デバイス：** RCサーボ、BLDCモータ（ESC）、ステッピングモータ、ロータリエンコーダ

| エミュレーション対象 | 方式 | 使用マイコン |
|-------------------|------|------------|
| PWM指令値の読取 | マイコンのタイマキャプチャ | Arduino / RP2040 / ESP32 |
| エンコーダパルス生成 | マイコンPWM出力 (A/B相) | RP2040 PIO（最適）|
| RCサーボ位置FB | PWMパルス幅読取→角度計算 | 汎用マイコン |
| モータ電流FB | DAC出力 (アナログ) | MCP4725 + マイコン |

---

## 4. UIAPduinoの位置づけと推奨マイコン選定

### 4.1 用途別マイコン比較

| マイコン/ボード | 価格帯 | USB CDC | USB UVC | Ethernet | CAN | I2C Slave | PIO | HILS適性 |
|---------------|--------|---------|---------|----------|-----|-----------|-----|---------|
| **UIAPduino CH32V003** | ~数百円 | × | × | × | × | △ | × | 限定的 |
| **Raspberry Pi Pico (RP2040)** | ~500円 | ○ | ○(低解像度) | ×(W5500追加可) | ×(MCP2515追加可) | ○(PIO) | ○ | **高** |
| **ESP32-S3** | ~700円 | ○ | △ | ×(内蔵なし) | ○(TWAI) | ○ | × | **高** |
| **Teensy 4.1** | ~4000円 | ○ | ○ | ○(内蔵) | ○(3ch) | ○ | × | **最高** |
| **STM32F407/H743** | ~1500円 | ○ | ○ | ○(内蔵) | ○(2-3ch) | ○ | × | **最高** |

### 4.2 UIAPduinoが適するケース

UIAPduinoは以下のような **限定的なユースケース** で有効：

1. **単純なUART中継**：UIAPduino + 外付けUSBシリアル変換チップでNMEA GPS等のシリアルセンサをエミュレート
2. **I2Cスレーブとしての簡易センサ模擬**：温度センサ、気圧センサ等のレジスタ読取りが単純なセンサ
3. **デジタルI/Oエミュレーション**：リミットスイッチ、フォトインタラプタ等の単純なON/OFF信号

### 4.3 標準インターフェースマイコン：RP2040 (Raspberry Pi Pico)

**入手性・価格・機能のバランスから、RP2040 (Raspberry Pi Pico) を本プロジェクトの標準HILSインターフェースマイコンとする。**

**選定理由：**
- **価格**: ~500円と非常に安価。複数台購入しても数千円で収まる
- **入手性**: 世界的に流通が安定しており、国内でも秋月電子・スイッチサイエンス等で即日入手可能
- **USB-CDC**: TinyUSBによりシリアルデバイスとして即座に認識される
- **PIO**: I2C/SPIスレーブ、エンコーダパルス生成など柔軟なプロトコル実装が可能
- **UVC対応**: TinyUSBのUVCデバイスクラスにより低解像度カメラとしても機能
- **エコシステム**: Arduino IDE / MicroPython / C SDK と豊富な開発環境
- **互換ボード**: RP2040-Zero, Seeed XIAO RP2040等の小型互換ボードも豊富

**CAN通信が必要な場合のみESP32-S3（TWAI内蔵）を併用** し、それ以外のインターフェースはRP2040で統一する方針とする。

**推奨構成：**

```
┌─────────────────────────────────────────────────┐
│              推奨HILSハードウェア構成               │
├──────────────────┬──────────────────────────────┤
│ シリアルセンサ     │ RP2040 × 1 (TinyUSB CDC)    │
│ (GPS, IMU, 超音波) │ → 実機PCでは /dev/ttyACM0   │
├──────────────────┼──────────────────────────────┤
│ I2C/SPIセンサ     │ RP2040 × 1 (PIO スレーブ)    │
│ (加速度計, ジャイロ)│                              │
├──────────────────┼──────────────────────────────┤
│ CANデバイス       │ ESP32-S3 + CANトランシーバ    │
│ (モータドライバ)   │ or RP2040 + MCP2515         │
├──────────────────┼──────────────────────────────┤
│ 3D LiDAR         │ RP2040 + W5500 (UDP送信)     │
│                  │ USBハブで複数台スケール可能    │
├──────────────────┼──────────────────────────────┤
│ カメラ            │ RP2040 UVC + MJPEG           │
│                  │ (640x480@17fps 〜             │
│                  │  1280x720@5-11fps)            │
├──────────────────┼──────────────────────────────┤
│ エンコーダ/PWM    │ RP2040 (PIO)                │
└──────────────────┴──────────────────────────────┘
```

---

## 5. ROSトピック→HILS変換の設計パターン

### 5.1 HILS Bridge Nodeの設計

各センサタイプに対応するROSノードを実装し、ROSトピックをプロトコル変換して物理インターフェースに出力する。

```python
# 設計パターンの概念コード (Python / rclpy)

class HilsBridgeNode(Node):
    """ROSトピック → 物理インターフェース変換の基本パターン"""
    
    def __init__(self):
        super().__init__('hils_bridge')
        
        # LiDAR: PointCloud2 → Velodyne UDPパケット
        self.create_subscription(PointCloud2, '/points', self.lidar_callback, 10)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # GPS: NavSatFix → NMEA over Serial
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
        
        # IMU: Imu → バイナリプロトコル over Serial
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
    def lidar_callback(self, msg):
        packets = self.pointcloud2_to_velodyne_packets(msg)
        for pkt in packets:
            self.udp_socket.sendto(pkt, (TARGET_IP, 2368))
    
    def gps_callback(self, msg):
        nmea = self.navsatfix_to_nmea(msg)
        self.serial_port.write(nmea.encode())
```

### 5.2 テスト戦略

HILSの導入により、以下の段階的テストが可能になる：

```
Level 0: 純粋シミュレーション (SILS)
  Gazebo → ROSトピック → アプリケーション
  ※ドライバ・物理層は未テスト

Level 1: ドライバレベルHILS
  シミュレーション → プロトコル変換 → 実機ドライバ → ROSトピック → アプリケーション
  ※物理通信経路はソフトウェアエミュレーション（v4l2loopback, localhost UDP等）

Level 2: 物理層HILS
  シミュレーション → マイコン → 物理ケーブル → 実機PC → ドライバ → ROSトピック → アプリ
  ※USB, UART, Ethernet, CAN等の物理経路を含む

Level 3: 完全HILS
  シミュレーション → 実機と同一のセンサ/アクチュエータI/F
  ※電源、EMI、コネクタを含む実環境に近い条件
```

---

## 6. 実現可能性のまとめ

### 6.1 各インターフェースの実現可能性一覧

| センサ/デバイス | インターフェース | エミュレーション方式 | 実現難易度 | 推奨HW | 実機ドライバとの互換性 |
|---------------|----------------|-------------------|----------|--------|-------------------|
| GPS受信機 | UART (NMEA) | マイコン USB-CDC + NMEA生成 | ★☆☆☆☆ | RP2040 | 完全互換 |
| シリアルIMU | UART (独自プロトコル) | マイコン USB-CDC + プロトコル生成 | ★★☆☆☆ | RP2040 | 完全互換（プロトコル実装次第） |
| I2C IMU (MPU6050等) | I2C スレーブ | マイコン PIO I2Cスレーブ | ★★★☆☆ | RP2040 (PIO) | 完全互換（レジスタマップ実装次第） |
| 3D LiDAR (Velodyne) | Ethernet UDP | RP2040 + W5500 UDP送信 | ★★☆☆☆ | RP2040 + W5500 | 完全互換 |
| 3D LiDAR (Ouster 16/32ch) | Ethernet UDP | RP2040 + W5500 UDP送信 | ★★☆☆☆ | RP2040 + W5500 | 完全互換 |
| 3D LiDAR (Ouster 128ch) | Ethernet UDP | 高速SPI or W6100必要 | ★★★★☆ | RP2040 + W6100 | 帯域要検証 |
| 3D LiDAR (Livox) | Ethernet UDP | RP2040 + W5500 UDP送信 | ★★★☆☆ | RP2040 + W5500 | SDK依存、要検証 |
| USBカメラ (MJPEG) | USB UVC | RP2040 UVC + MJPEG パススルー | ★★☆☆☆ | RP2040 | 完全互換 (640x480@17fps, 720p@5-11fps) |
| USBカメラ (高fps要) | USB UVC | STM32H7 / Teensy 4.x UVC | ★★★☆☆ | STM32H7 / Teensy | 完全互換 (1080p@30fps) |
| GigE Visionカメラ | Ethernet GigE Vision | GigESim / SW | ★★★☆☆ | PC + SDK | 互換 |
| CANモータドライバ | CAN bus | マイコン + CANトランシーバ | ★★★☆☆ | ESP32 / RP2040+MCP2515 | プロトコル実装次第 |
| RCサーボ | PWM | マイコン PWM出力/入力 | ★☆☆☆☆ | 汎用マイコン | N/A（アナログ信号） |
| エンコーダ | パルス (A/B相) | マイコン PIO | ★★☆☆☆ | RP2040 (PIO) | 完全互換 |
| 力覚センサ | アナログ / SPI | DAC出力 / SPIスレーブ | ★★★☆☆ | RP2040 + DAC | 方式依存 |

### 6.2 結論

1. **安価なマイコンを活用したHILSは十分に実現可能**であり、特にシリアルセンサ（GPS, IMU）とEthernet接続センサ（3D LiDAR）については既存のROSドライバとの完全互換を維持したHILSが構築できる

2. **UIAPduino単体ではHILSへの適用は限定的**（USB Low-Speed、リソース制約）だが、外付けUSBシリアル変換チップとの組み合わせや、単純なI2Cスレーブとしての利用は可能。UIAPduino 2台構成でのシリアルHILSは、USB-シリアル変換チップを介する構成であれば実現可能

3. **RP2040 (Raspberry Pi Pico) を標準HILSインターフェースマイコンとして採用**：USB-CDC対応、PIOによる柔軟なプロトコルエミュレーション、TinyUSB UVCサポート、入手性・価格ともに優秀。CAN通信が必要な場合のみESP32-S3を併用

4. **USBカメラのHILSはRP2040 UVC + MJPEGパススルー方式を推奨**：シミュレーションPC側でJPEGエンコードし、RP2040はUVC MJPEGフレームとしてパススルー出力する。USB Full-Speed (12Mbps) でも640x480@17fps、1280x720@5-11fpsが達成可能で、多くのロボット用途に実用的。市販USBカメラでも高解像度時にfpsを落とす設計は一般的であり、違和感のない方式

5. **3D LiDARのHILSはRP2040 + W5500でEthernet出力**：パケットフォーマットが公開されているため完全互換のエミュレーションが可能。USBハブ経由でRP2040を追加するだけで複数LiDAR構成にもスケール可能であり、PC側のLANポート不足問題を回避できる

6. **ros2_controlの`hardware_interface`抽象化を活用**することで、HILSと実機の切替をソフトウェア設定のみで実現できる設計が望ましい

---

## 参考資料

- [NVIDIA Hardware-in-the-Loop with Jetson](https://developer.nvidia.com/blog/design-your-robot-on-hardware-in-the-loop-with-nvidia-jetson/)
- [PX4 HITL Simulation](https://docs.px4.io/main/en/simulation/hitl.html)
- [ros2_control Mock Components](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)
- [TinyUSB - Open Source USB Stack](https://github.com/hathach/tinyusb)
- [UIAPduino 公式サイト](https://www.uiap.jp/en/uiapduino/)
- [Velodyne VLP-16 Packet Structure](https://velodynelidar.com/wp-content/uploads/2019/09/63-9276-Rev-C-VLP-16-Application-Note-Packet-Structure-Timing-Definition.pdf)
- [Ouster Sensor Data Format](https://static.ouster.dev/sensor-docs/image_route1/image_route2/sensor_data/sensor-data.html)
- [Livox SDK Communication Protocol](https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol)
- [v4l2loopback](https://github.com/umlaeute/v4l2loopback)
- [GigESim - GigE Vision Camera Emulator](https://www.ab-soft.com/gigesim.php)
- [Robotics Knowledgebase: HIL/SIL Testing](https://roboticsknowledgebase.com/wiki/system-design-development/In-Loop-Testing/)
