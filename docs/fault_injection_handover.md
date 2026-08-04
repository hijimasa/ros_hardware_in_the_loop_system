# 故障注入HILS 引継ぎ資料

最終更新: 2026-07-29

本資料は、故障注入HILS基盤の現状と、**ハードウェアが必要なため未実施の作業**を別環境で実施するための手順をまとめたものである。設計の背景は[fault_injection_implementation_policy.md](fault_injection_implementation_policy.md)(以下「方針書」)、正常系の検証手順は[hils_verification_guide.md](hils_verification_guide.md)を参照。

---

## 1. 現状サマリ

### 1.1 実装済み(ソフトウェアのみで完結する範囲はすべて完了)

| コンポーネント | 場所 | 状態 |
| --- | --- | --- |
| 故障パイプライン(9種+プロトコル故障2種+ファームウェア協調7種+netem) | `hils_bridge_base/fault_injection/` | 実装・単体テスト済み |
| デバイス状態機械(電源断・再起動) | `hils_bridge_base/device_state/` | 実装・単体テスト済み |
| YAMLシナリオランナー | `hils_bridge_base/scenario/` | 実装・E2E済み |
| 試験オラクル(自動合否判定+JSON/JUnit XML+rosbag/PCAP記録) | `hils_bridge_base/observation/`, `reporting/` | 実装・E2E済み |
| サービスAPI定義 | `hils_bridge_interfaces/` | 実装済み |
| シナリオ例・launch | `hils_bringup/scenarios/`, `launch/` | 実装済み |
| CI(ビルド+単体92件+GPS E2E、nightly) | `.github/workflows/hils_tests.yml` | 実装済み(初回実行の確認は未) |

対応エミュレータ: Velodyne VLP-16 / Livox Mid-360 / Ouster OS1(HTTP故障含む)/ Hokuyo YVT-35LX(VSSP 2.1 TCP)/ NMEA GPS / WT901 IMU / UVCカメラ(PC側のみ)。`serial_write()`/`send_udp()`を使う全ブリッジが自動的に故障注入対応になる。

### 1.2 実ドライバ検証済み

| 対象 | 検証内容 | 結果 |
| --- | --- | --- |
| livox_ros_driver2 1.2.6 | 正常系(bag→エミュレータ→ドライバ、レンジ中央値完全一致)、電源断→再起動の自動復帰、reset時の復帰不能(負例) | 方針書22.3節 |
| ouster_ros 0.14.1 | 正常系(HTTPメタデータ+UDP)、設定APIのみ無応答中のストリーム継続 | 方針書22.2節 |
| nmea_navsat_driver | チェックサム不正で/fix停止・ドライバ生存・復帰(PTYペアおよびFT234X実機経路、オラクル自動判定PASS) | 方針書22.1節Phase 4 |
| witmotion_ros (ElettraSciComp) | wt901_checksum故障で/imu停止・ドライバ生存・復帰(PTYペアおよびFT234X実機経路、オラクル自動判定PASS) | 方針書22.1節Phase 4 |
| UVCホスト(Linux uvcvideo) | Picoペア実機で正常系(映像+解像度逆コマンド)、drop/corrupt/delayのフリーズ・回復挙動 | 方針書22.2節 |
| urg3d_node2 (Hokuyo公式) | 正常系(VSSP 2.1ハンドシェイク→10Hz点群)、計測ストリーム3秒停止からの自動復帰(オラクル自動判定PASS)。バイト破損では**ドライバがsegfault**(発見事項、方針書22.5節) | 方針書22.5節 |

### 1.3 主要な発見事項

**Livox SDK2は一度設定したデバイスを再設定しない**(ハートビートも無い)。実機MID-360が電源断から復帰できるのはデバイス側がWorkModeを保持するため。エミュレータの`reboot_config_policy`(preserve/reset)でこの挙動を選択でき、`reset`はドライバの再接続性欠陥を露見させる故障シナリオとして使える。詳細は方針書22.3節。

---

## 2. 環境の再現方法

### 2.1 前提

- dockerイメージ: `docker/build_docker_image.sh`でビルド(`<user>/ros-jazzy-hils`)
- コンテナ: `docker compose -f docker/docker-compose.yml up -d`
  (sim=192.168.100.201/domain42、robot=192.168.100.100/domain43)
- シミュレーションデータ: MID-360実機bag
  `~/git_ws/3D_LIDAR_TEST/ros_ws/exp1_bags/`(壁2/5/10/15/20m、
  `/livox/lidar`=PointCloud2、`/livox/imu`=Imu)。
  コンテナへは compose override でマウントする(下記)

```yaml
# compose override例(bagとテスト資材のマウント)
services:
  sim:
    volumes:
      - <bagディレクトリ>:/home/ubuntu/bags:ro
```

- robotコンテナのドライバ: livox_ros_driver2はソースビルドが必要
  (`cp package_ROS2.xml package.xml && colcon build --packages-select
  livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy`。
  `HUMBLE_ROS`ではなく`DISTRO_ROS`である点に注意)。
  ouster_ros / nmea_navsat_driver はaptで入る
- **コンテナ再作成でrobot側のビルド成果物は消える**(`~/colcon_ws/install`は
  マウント外)。`compose down`後は再ビルドすること

### 2.2 基本的な試験の流れ

```bash
# sim側: エミュレータ + データ源
ros2 run hils_bridge_lidar_livox_mid360 livox_emulator_node --ros-args \
  -p device_ip:=192.168.100.201 -p host_ip:=192.168.100.100
ros2 bag play /home/ubuntu/bags/exp1_wall_d10m_* --loop

# robot側: 実ドライバ(MID360_config.jsonのIPをエミュレータに向ける)

# sim側: オラクル(先) → ランナー(後)。オラクルの終了コードが合否
ros2 run hils_bridge_base scenario_oracle --ros-args \
  -p scenario_file:=<scenario.yaml> -p observe_domain_id:=43 \
  -p output_dir:=/tmp/reports -p record_bag:=true
ros2 run hils_bridge_base scenario_runner --ros-args \
  -p scenario_file:=<scenario.yaml>
```

ハードウェア不要のE2Eは各1コマンド(ビルド済みws前提):
- GPS: `bash tools/run_gps_e2e.sh`(`nmea_navsat_driver`+socat)
- WT901: `bash tools/run_imu_e2e.sh`(`witmotion_ros`のビルドが必要)
- Velodyne: `bash tools/run_velodyne_e2e.sh`(`velodyne-driver`/
  `velodyne-pointcloud`。ループバックUDP、点群源は合成壁または
  `E2E_BAG=<bagディレクトリ>`)
- Ouster: `bash tools/run_ouster_e2e.sh`(`ouster-ros`。HTTP port 80の
  bind権限が必要=rootまたは`ip_unprivileged_port_start=80`)
- Hokuyo YVT-35LX: `bash tools/run_yvt35lx_e2e.sh`(`urg3d_node2`の
  ソースビルドが必要=`git clone --recursive
  https://github.com/Hokuyo-aut/urg3d_node2`+`ros-<distro>-laser-proc`。
  ループバックTCP、点群源は合成壁または`E2E_BAG=<bagディレクトリ>`。
  既定シナリオは`yvt35lx_blackout_001`、`E2E_SCENARIO=<名前>`で切替)

---

## 3. 未実施作業(ハードウェアが必要)

### 3.1 FT234X×2でのGPS/WT901実機回帰(2026-07-29完了)

**完了済み。** FT234X×2クロス接続(`/dev/ttyUSB0`↔`/dev/ttyUSB1`)を
`docker/docker-compose.serial.yml`(新規)でsimコンテナへパススルーし、
GPS・WT901とも実ポートでPTY版と同一の3件PASSを確認した。
結果詳細は方針書22.1節Phase 4・22.2節を参照。

再現方法(1コマンド、simコンテナ内):

```bash
# GPS (9600 baud)
E2E_SERIAL_BRIDGE=/dev/ttyUSB0 E2E_SERIAL_DRIVER=/dev/ttyUSB1 \
  bash tools/run_gps_e2e.sh
# WT901 (115200 baud、witmotion_rosのビルドが必要)
E2E_SERIAL_BRIDGE=/dev/ttyUSB0 E2E_SERIAL_DRIVER=/dev/ttyUSB1 \
  bash tools/run_imu_e2e.sh
```

環境変数を省略すると従来どおりsocat PTYペアで動く(ハードウェア不要)。
WT901用に`scenarios/imu/wt901_checksum_error_001.yaml`と
`tools/run_imu_e2e.sh`を追加した。socatはdockerイメージに追加済み
(旧イメージを使う場合は`apt install socat`)。

注意: `witmotion_ros`の既定`timeout_ms: 150`はブリッジ起動との
レースでSUSPENDED→segfaultに至るため、`run_imu_e2e.sh`は設定パッチで
2000msへ拡大している(方針書22.2節の発見事項参照)。

### 3.2 UVCカメラ実機経路の回帰(2026-07-30完了)

**完了済み。** Picoペア実機で正常系(映像出力+解像度逆コマンド)と
故障注入(drop/corrupt/delay)を確認した。結果と発見事項の詳細は
方針書22.2節を参照。README脚注`[^fault-refactor-note]`も解消済み。

要点:
- 3故障ともUVCホストには「映像フリーズ」として現れる(Pico#2が
  最終フレームを再送し続けるためUVCストリーム自体は止まらない)。
  corruptはPico#1のフレームチェックサム検証で棄却されるため
  壊れJPEGはホストに届かない(当初期待の「corrupt=壊れJPEG」は
  プロトコル設計上発生しない、が正しい挙動)
- Pico#1のファームウェア書き換えはBOOTSELボタン不要
  (1200baudタッチ→`/media/$USER/RPI-RP2`へuf2コピー)
- 相手ファームウェアがCDCを読まない場合にブリッジが無限停止する
  問題はSerialBridgeBaseの`write_timeout_sec`(既定1.0s)で対策済み
- コンテナへのデバイス受け渡しは`docker/docker-compose.uvc.yml`
  (`/dev/ttyACM0`+`/dev/video0-1`)を使用
- 追記(2026-07-31): `usb_cam`(`pixel_format:=mjpeg2rgb`)での取得に
  対応した。UVCディスクリプタのDiscrete化(要Pico#2再書き込み)と
  ブリッジの4:2:2 JPEGエンコード化の2点が必要だった。詳細は
  方針書22.2節「UVCホスト互換性の改善」を参照

### 3.3 Phase 5: ファームウェア協調故障(UVC・I2Cとも2026-07-31完了)

**UVC側は実装・実機確認済み。** 0x50番台プロトコル、PC側
`FirmwareFault`クラス群(`uvc_usb_detach`/`uvc_frame_drop`/
`uvc_partial_frame`)、`rp2040_camera_uvc`の`fault_handler.c`を追加し、
detach(自動再列挙)・フレーム欠損(フリーズ)・不完全フレーム
(壊れJPEGのホスト到達)を実機確認した。結果詳細は方針書22.1節
Phase 5を参照。注入は通常の`~/inject_fault`サービスから
`fault_type: uvc_usb_detach`等で行う(専用APIなし)。

運用メモ:
- Pico#2は`RESET_BOOTSEL`(0x5F)対応済み。以後の書き換えは
  `frame_protocol.build_reset_bootsel()`をブリッジ経由で送れば
  BOOTSELボタン不要(RPI-RP2マウント→uf2コピー)
- `uvc_frame_drop`はホスト側drop故障と違いUART帯域は消費し続ける
  (帯域不足相当の欠損)。ホスト側corruptと違い`uvc_partial_frame`は
  壊れJPEGが実際にホストへ届く(Pico#1のチェックサム棄却を受けない)

**I2C(MPU-6050)側も2026-07-31完了。** Pico×2クロス構成
(スレーブ=MPU-6050エミュレータ、マスター=新規`rp2040_mpu6050_reader`が
実ドライバ代役)で、`i2c_nack`/`i2c_response_delay`/`i2c_reg_freeze`/
`i2c_who_am_i`の4故障を実機確認した。結果詳細は方針書22.1節Phase 5。
配線は[hils_verification_guide.md](hils_verification_guide.md)5節
(プルアップ用3.3Vはスレーブ側GPIO 3=物理5番ピンから供給)。

運用メモ:
- 両Picoともリモート書き換え可(スレーブ=1200baudタッチまたは0x5F、
  マスター=1200baudタッチ)
- マスターリーダーのCDCログ(`[STAT]`/`[WHO]`/`[ERR]`/`[REINIT]`)が
  ホスト側の合否判定入力。デバイスは`/dev/serial/by-id/`で特定する
  (CDC持ちPicoが複数あるとttyACM番号は不定)
- RP2040 I2Cマスターはスレーブ消失タイムアウト後にコントローラが
  固着する。リーダーはSCL 9クロック+再初期化のバスリカバリを
  100連続エラー毎に実行する(方針書22.1節Phase 5の発見事項)

### 3.4 その他の未検証項目

- エンコーダ(`hils_bridge_encoder_quadrature`): ビルドは修正済みだが
  実機未検証(README: Implemented, unverified)。RP2040+実コントローラの
  エンコーダ入力で確認
- レベル3電源断(方針書8.1): USBリレー/管理スイッチによる物理切断。
  対応ハードウェア導入後、`set_device_state`と同じシナリオアクションから
  制御できるよう拡張する
- 2台PC構成での時刻同期(chrony/PTP)を伴う試験: 方針書17.8節。
  現状の検証はすべて1PC(docker)内

---

## 4. ソフトウェア側の残タスク(ハードウェア不要だが未実施)

**すべて解消済み(2026-07-31時点で§4の残タスクなし)。**

解消済み:

- オラクル判定タイプ`invalid_message_not_published`(2026-07-31):
  内容検査を実装。使用例は`gps_checksum_error_001.yaml`の
  latitude/longitude境界検査、仕様は`observation/expectations.py`の
  docstringを参照
- netemバックエンド(2026-07-31): `fault_type: netem`として実装
  (方針書22.1節Phase 6参照)。CAP_NET_ADMIN+iproute2+tcへのsetcapが
  前提(compose/Dockerfileに反映済み。**既存イメージ・コンテナは
  再ビルド/再作成が必要**)

- CIの初回実行確認(2026-07-31): push後のGitHub Actionsでエラーなし
- Velodyne/Ouster故障シナリオのオラクル付き定期実行化(2026-07-31):
  `run_velodyne_e2e.sh`/`run_ouster_e2e.sh`としてループバック1コマンド化
  しCIにも追加(方針書22.1節Phase 6参照)。点群源はデフォルト合成壁、
  `E2E_BAG`で実bag切替(livox_20251219_175610でPASS確認済み)。
  未対応シナリオ: `velodyne_corrupt_drop_001`/`velodyne_delay_jitter_001`/
  `velodyne_reboot_001`/`livox_reboot_001`はスクリプト化していない
  (SCENARIOを差し替えれば同じ枠組みで実行可能)

## 5. 運用メモ

- シナリオ形式・期待条件の仕様: `hils_bridge_base/scenario/scenario_loader.py`
  と`observation/expectations.py`のdocstringが正
- 故障の種類とパラメータ: `fault_injection/`各ファイルのdocstring
- オラクルのレポート: JSON(詳細)とJUnit XML(CI)。`record_bag:=true`で
  観測トピックのrosbag、`record_pcap:=true`+tcpdump導入+CAP_NET_RAWで
  PCAPも保存される
- 既知の落とし穴:
  - bag再生のstampは古いので`maximum_message_age`は使えない
  - ランナーとオラクルは同一ホストで起動する(オラクルはランナーの
    サービスから経過時間を取得して時計を合わせる)
  - `pkill -f`はコマンドライン全文にマッチするため、スクリプト内で
    エミュレータ名を含むheredocを使うと自爆する(`[l]ivox`等で回避)
