# 故障注入HILS 引継ぎ資料

最終更新: 2026-07-29

本資料は、故障注入HILS基盤の現状と、**ハードウェアが必要なため未実施の作業**を別環境で実施するための手順をまとめたものである。設計の背景は[fault_injection_implementation_policy.md](fault_injection_implementation_policy.md)(以下「方針書」)、正常系の検証手順は[hils_verification_guide.md](hils_verification_guide.md)を参照。

---

## 1. 現状サマリ

### 1.1 実装済み(ソフトウェアのみで完結する範囲はすべて完了)

| コンポーネント | 場所 | 状態 |
| --- | --- | --- |
| 故障パイプライン(9種+プロトコル故障2種) | `hils_bridge_base/fault_injection/` | 実装・単体テスト済み |
| デバイス状態機械(電源断・再起動) | `hils_bridge_base/device_state/` | 実装・単体テスト済み |
| YAMLシナリオランナー | `hils_bridge_base/scenario/` | 実装・E2E済み |
| 試験オラクル(自動合否判定+JSON/JUnit XML+rosbag/PCAP記録) | `hils_bridge_base/observation/`, `reporting/` | 実装・E2E済み |
| サービスAPI定義 | `hils_bridge_interfaces/` | 実装済み |
| シナリオ例・launch | `hils_bringup/scenarios/`, `launch/` | 実装済み |
| CI(ビルド+単体92件+GPS E2E、nightly) | `.github/workflows/hils_tests.yml` | 実装済み(初回実行の確認は未) |

対応エミュレータ: Velodyne VLP-16 / Livox Mid-360 / Ouster OS1(HTTP故障含む)/ NMEA GPS / WT901 IMU / UVCカメラ(PC側のみ)。`serial_write()`/`send_udp()`を使う全ブリッジが自動的に故障注入対応になる。

### 1.2 実ドライバ検証済み

| 対象 | 検証内容 | 結果 |
| --- | --- | --- |
| livox_ros_driver2 1.2.6 | 正常系(bag→エミュレータ→ドライバ、レンジ中央値完全一致)、電源断→再起動の自動復帰、reset時の復帰不能(負例) | 方針書22.3節 |
| ouster_ros 0.14.1 | 正常系(HTTPメタデータ+UDP)、設定APIのみ無応答中のストリーム継続 | 方針書22.2節 |
| nmea_navsat_driver | チェックサム不正で/fix停止・ドライバ生存・復帰(PTYペア、オラクル自動判定PASS) | 方針書22.1節Phase 4 |

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

ハードウェア不要のGPS E2Eは1コマンド: `bash tools/run_gps_e2e.sh`
(ビルド済みws・`nmea_navsat_driver`・socatが必要)。

---

## 3. 未実施作業(ハードウェアが必要)

### 3.1 FT234X×2でのGPS/WT901実機回帰

PTYペアでのE2Eは完了しているが、**USBシリアルのOSスタック(FTDIドライバ、
実ボーレート、バッファリング)を経由した検証は未実施**。

必要機材: FT234X×2をクロス接続(TX↔RX)、PC1台(または2台)。

手順:
1. `tools/run_gps_e2e.sh`を参考に、socatの代わりに実ポートを使う:
   ブリッジ側`serial_port:=/dev/ttyUSB0`、ドライバ側`port:=/dev/ttyUSB1`、
   両方`baudrate/baud:=9600`
2. シナリオ`gps_checksum_error_001.yaml`をランナー+オラクルで実行
3. 期待: PTY版と同じ(3件PASS)。差が出る場合はボーレート起因の
   フレーム分割(1 writeが複数readに割れる)が疑い所
4. WT901: `witmotion_ros`をビルドし(`libqt5serialport5-dev`必要、
   docker imageには導入済み)、`wt901_checksum`故障で同様の試験。
   シナリオは`gps_checksum_error_001.yaml`を雛形に`target: /hils_imu_bridge`、
   `fault_type: wt901_checksum`、トピック`/imu`系へ書き換え
5. 完了したら方針書22.2節の表を更新

### 3.2 UVCカメラ実機経路の回帰(§22.2の未確認事項)

SerialBridgeBase移行後、Picoペア経由の実機確認が未実施。

必要機材: RP2040×2(`rp2040_camera_uvc_spi_sender`+`rp2040_camera_uvc`
ファームウェア書き込み済み)、SPI配線、ホストPC。

手順:
1. 正常系: `/image_raw`を流し、ホストPCの`/dev/video0`に映像が出ること、
   解像度逆コマンド(`ros2 param`への反映)を確認
2. 故障注入: `/hils_uvc_bridge/inject_fault`でdrop(映像停止)、
   corrupt(壊れJPEG)、delay(フレーム遅延)を注入し、UVCホスト側の
   挙動(フリーズ/回復)を確認
3. 完了したら方針書22.2節とREADMEの脚注`[^fault-refactor-note]`を解消

### 3.3 Phase 5: USBファームウェア協調故障

方針書9.3節・9節冒頭の表のとおり、USB detach/descriptor異常は
ファームウェア側の実装が必要。

実装方針(方針書9節):
1. `firmware/common/include/hils_frame_protocol.h`に故障注入コマンド
   0x50番台のメッセージタイプを追加(例: `0x50`=fault set、
   `0x51`=fault clear、`0x52`=fault ack)
2. PC側: `FaultInjectionController`から未知fault_typeをファームウェアへ
   転送するブリッジ固有Faultクラスを作り`register_fault_class()`で登録
3. ファームウェア側(`rp2040_camera_uvc`): `tud_disconnect()`/
   `tud_connect()`によるソフトウェアdetach(リレー不要)、フレーム欠損、
   不完全フレーム
4. I2C(MPU-6050)側も同様: NACK、応答遅延、レジスタ固定は
   `rp2040_imu_invensense_mpu6050/src/main.c`のI2Cスレーブ処理に実装

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

- オラクル判定タイプ`invalid_message_not_published`(不正データが
  トピックに現れないことの内容検査。メッセージ内容の検証が必要で、
  現状の到着時刻ベース観測では判定できない)
- CIの初回実行確認(pushすると`.github/workflows/hils_tests.yml`が動く。
  ros:jazzyコンテナ内のsocat/E2EがGitHub Actions環境で通るかは未確認)
- netemバックエンド(方針書4.5節): 統計的トランスポート故障のtc qdisc
  連携は設計のみで未実装
- Velodyne/Ousterの故障シナリオ(`velodyne_*.yaml`、
  `ouster_http_error_001.yaml`)のオラクル付き定期実行化
  (点群データ源が必要。MID-360 bagのトピックリマップで流用可)

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
