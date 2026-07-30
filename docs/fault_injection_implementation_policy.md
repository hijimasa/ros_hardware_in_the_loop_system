# 公開プロトコルベース故障注入HILS 実装方針

## 1. 文書の目的

本資料は、`ros_hardware_in_the_loop_system`に対して、公開されている機器通信プロトコルの範囲内で故障注入機能を追加するための実装方針を定める。

本システムでは、実センサ内部の制御基板や実モータドライバ内部へ接続することを目的としない。ROS 2用ドライバが利用している公開プロトコルおよび公開インターフェースを模擬し、実機相当PC上で動作する実際のドライバ、ROS 2ノード、監視処理、復旧処理を検証対象とする。

主な検証対象は以下とする。

* デバイス電源断相当の通信消失
* USB、シリアル、Ethernetなどの切断
* 応答遅延および周期揺らぎ
* パケット欠損
* パケット重複
* パケット順序入れ替え
* 不正なチェックサムやCRC
* 不正な長さ、値、識別子
* タイムスタンプ異常
* デバイス再起動相当の状態遷移
* 設定要求への異常応答
* 通信復旧後の再接続および再初期化
* ROS 2ドライバや上位アプリケーションの異常検出
* watchdogや監視ノードによる安全側への遷移

---

## 2. 適用範囲

### 2.1 対象とする範囲

本システムでは、以下の境界までを試験対象とする。

```text
シミュレータ
    ↓ ROS 2トピック
HILS Bridge / Device Emulator
    ↓ 公開された機器プロトコル
USB / UART / Ethernet / CAN / I2C / SPI
    ↓
実機相当PCまたは実制御マイコン
    ↓
実際のROS 2ドライバまたは公開プロトコル利用プログラム
    ↓
ROS 2アプリケーション、監視処理、制御処理
```

試験対象に含めるものは以下である。

* 実機相当PC
* 実際のOS、カーネル、USBスタック、ネットワークスタック
* 実際のROS 2ディストリビューション
* 実際のRMW実装
* 実際のROS 2ドライバ
* 実際のlaunchファイルおよび設定ファイル
* 実際の上位アプリケーション
* 公開仕様に基づくデバイスプロトコル
* 公開仕様に基づくデバイス状態遷移
* 公開インターフェース上で観測可能な故障状態

### 2.2 対象外とする範囲

以下は本システムの対象外とする。

* 実センサ素子とセンサ制御基板間の非公開通信
* 実モータとモータドライバ間の非公開通信
* 非公開レジスタ、非公開診断プロトコル
* ベンダ独自の内部ファームウェア処理
* モータの電流制御ループそのもの
* センサ内部のアナログフロントエンド
* 実デバイス内部の熱設計や電源回路
* 実デバイス内部の部品故障
* EMI、ESD、サージなどの物理的耐性試験
* 安全PLCや安全機器の認証試験
* 機能安全規格への適合性そのものの証明

非公開仕様の解析、リバースエンジニアリング、実機内部信号の調査を前提とする機能は実装しない。

---

## 3. システムの位置づけ

本システムは、以下の試験レベルを対象とする。

| レベル                            | 試験内容                      | 本システムでの扱い     |
| ------------------------------ | ------------------------- | ------------- |
| SILS                           | ROS 2トピック上での正常・異常データ試験    | 対応            |
| PILS                           | 実機相当PC上で実プログラムを実行         | 対応            |
| Driver-in-the-Loop             | 実際のROS 2ドライバへ機器プロトコルを入力   | 主要対象          |
| Protocol-in-the-Loop           | 公開プロトコルの正常・異常状態を試験        | 主要対象          |
| Physical-Interface-in-the-Loop | USB、UART、Ethernetなどを実際に経由 | 対応            |
| Controller HIL                 | 実制御器を閉ループ接続               | 公開I/Fで可能な場合のみ |
| 内部基板HIL                        | 非公開内部信号へ接続                | 対象外           |
| Safety HIL                     | 安全PLCやSTOを含む安全機能試験        | 将来拡張、別系統で実施   |

プロジェクトの説明では、単に「完全なHILS」と表現するのではなく、次の表現を基本とする。

> 公開プロトコルと物理通信インターフェースを利用した、ROS 2向けDriver-in-the-LoopおよびProtocol-in-the-Loop試験基盤

---

## 4. 基本設計方針

### 4.1 公開プロトコルのみを利用する

デバイスエミュレータは、以下のいずれかに基づいて実装する。

* ベンダが公開している通信仕様書
* 公開SDK
* オープンソースの公式ドライバ
* ベンダが公開しているAPI仕様
* USB標準デバイスクラス仕様
* NMEA、CANopenなどの公開標準
* 公開されているレジスタマップ
* 公開されている設定・診断コマンド

公開ドライバのソースコードを参考にする場合でも、特定バージョンの実装だけに依存せず、可能な限り公開仕様との対応関係を記録する。

### 4.2 プロトコルエミュレータを状態機械として実装する

単純にROS 2メッセージをパケットへ変換するだけでなく、デバイスの外部観測可能な状態を状態機械として表現する。

標準状態は以下とする。

```text
POWER_OFF
    ↓
BOOTING
    ↓
DISCOVERABLE
    ↓
CONFIGURING
    ↓
READY
    ↓
STREAMING
```

異常状態として以下を用意する。

```text
DEGRADED
COMMUNICATION_FAULT
CONFIGURATION_FAULT
INTERNAL_ERROR
REBOOTING
POWER_OFF
```

すべてのデバイスで同じ状態を必須とする必要はないが、共通基盤では同等の概念を扱えるようにする。

### 4.3 正常データ生成と故障注入を分離する

以下の処理を独立した層として実装する。

```text
ROS 2入力
    ↓
デバイスデータモデル
    ↓
正常プロトコルデータ生成
    ↓
故障注入
    ↓
通信インターフェース出力
```

故障注入を各デバイス固有コードへ直接埋め込まず、可能な範囲で共通コンポーネントとして実装する。

### 4.4 故障注入は再現可能であること

すべての故障シナリオは、設定ファイルから再現できるようにする。

乱数を使用する場合はシード値を記録する。

試験結果には最低限、以下を保存する。

* シナリオID
* シナリオ設定
* 乱数シード
* リポジトリのcommit ID
* 対象ドライバのバージョン
* ROS 2ディストリビューション
* RMW実装
* OSおよびカーネルバージョン
* 試験開始・終了時刻
* 注入した故障
* 観測結果
* 合否判定
* rosbag
* エミュレータログ
* ドライバログ

### 4.5 統計的故障と決定的故障を実装層で区別する

トランスポート層の統計的故障(遅延、ジッタ、欠損、重複、順序入れ替え、ビット破損)は、Linux標準のnetem(tc qdisc)をバックエンドとして利用できる。Docker構成ではコンテナの仮想インターフェースへ適用するだけで済み、自前実装の量と時刻精度の負担を削減できる。

ただしnetemには以下の制約がある。

* シードによる決定的な再現ができない
* プロトコルフィールドを認識しない
* 特定のパケットだけを対象にできない

したがって以下のように実装層を使い分ける。

| 故障の種類 | 実装層 |
| --- | --- |
| 統計的なトランスポート故障 | netemバックエンド(選択可能) |
| 決定的故障、フィールド認識が必要な故障 | 自前の故障注入パイプライン |
| プロトコル状態遷移をともなう故障 | デバイス状態機械 |

netemを使用した試験では、適用したqdisc設定を試験結果に記録する。

---

## 5. 提案アーキテクチャ

```text
┌──────────────────────────────────────────────┐
│ シミュレーションPC                           │
│                                              │
│  Gazebo / Isaac Sim / Unity / rosbag         │
│                  │                           │
│                  ▼                           │
│  Device Data Model                           │
│                  │                           │
│                  ▼                           │
│  Protocol Encoder                            │
│                  │                           │
│                  ▼                           │
│  Fault Injection Pipeline                    │
│    - delay                                   │
│    - drop                                    │
│    - corrupt                                 │
│    - duplicate                               │
│    - reorder                                 │
│    - freeze                                  │
│    - reboot                                  │
│    - disconnect                              │
│                  │                           │
│                  ▼                           │
│  Transport Adapter                           │
│    - UDP/TCP                                 │
│    - Serial                                  │
│    - USB                                     │
│    - CAN                                     │
│    - I2C/SPI                                 │
└──────────────────┬───────────────────────────┘
                   │ 物理または仮想I/F
                   ▼
┌──────────────────────────────────────────────┐
│ 実機相当PC                                   │
│                                              │
│  実ROS 2ドライバ                             │
│          │                                   │
│          ▼                                   │
│  ROS 2アプリケーション                       │
│          │                                   │
│          ▼                                   │
│  Watchdog / Diagnostics / Safety Supervisor  │
└──────────────────────────────────────────────┘
```

---

## 6. ソフトウェア構成案

```text
ros2_hils_bridge/
├── hils_bridge_base/
│   ├── protocol/
│   ├── transport/
│   ├── device_state/
│   ├── fault_injection/
│   ├── scenario/
│   ├── observation/
│   └── reporting/
│
├── hils_bringup/
│   ├── launch/
│   ├── config/
│   └── scenarios/
│
├── hils_bridge_lidar/
├── hils_bridge_camera/
├── hils_bridge_gps/
├── hils_bridge_imu/
├── hils_bridge_actuator/
├── hils_bridge_encoder/
└── hils_bridge_can/
```

### 6.1 `hils_bridge_base/fault_injection`

共通の故障注入処理を実装する。

```text
fault_injection/
├── fault_base.py
├── pipeline.py
├── scheduler.py
├── delay_fault.py
├── jitter_fault.py
├── drop_fault.py
├── duplicate_fault.py
├── reorder_fault.py
├── corruption_fault.py
├── freeze_fault.py
├── disconnect_fault.py
├── reboot_fault.py
└── rate_fault.py
```

### 6.2 `hils_bridge_base/device_state`

デバイス状態を共通管理する。

```text
device_state/
├── state.py
├── state_machine.py
├── transition.py
└── lifecycle_adapter.py
```

### 6.3 `hils_bridge_base/scenario`

YAMLシナリオの読み込み、時刻制御、イベント実行を担当する。

```text
scenario/
├── scenario_loader.py
├── scenario_validator.py
├── scenario_runner.py
├── event_scheduler.py
└── condition_trigger.py
```

### 6.4 `hils_bridge_base/observation`

試験対象の状態を観測する。

* ROS 2トピック
* ROS 2 diagnostics
* ノード状態
* Lifecycle状態
* トピック周期
* メッセージタイムスタンプ
* ドライバプロセス
* デバイスファイル
* ネットワークリンク
* 必要に応じてGPIO入力

#### 観測処理の配置

試験対象である実機相当PCへ観測エージェントを追加すると、実機との環境差分が生じる。そのため観測処理はシミュレーションPC側へ配置し、実機相当PCには観測用プロセスを追加しないことを原則とする。

シミュレーションPC上に観測専用のコンテナまたはプロセスを用意し、実機相当PC側のROS_DOMAIN_IDへ読み取り専用で参加して、トピック、diagnostics、ノードグラフを観測する。

構成上の注意は以下のとおり。

* 観測プロセスは購読とグラフ参照のみ行い、publishしない
* 観測用DDS通信が、エミュレートされたセンサ通信と同じリンクの帯域を消費しないよう、観測専用ネットワークを分離する
  * Docker構成では観測用ネットワークを追加する
  * 実機2台構成では第2のEthernetポートを使用する
* 点群など高帯域トピックの購読は必要最小限とする
* 観測参加によるDDS discoveryや配信負荷は実機相当PCへの介入として扱い、試験条件に記録する(rvizを実ロボットへ接続する場合と同程度の介入である)

ドライバプロセスの死活は、以下を区別して判定する。

* DDS participantの消失:プロセスクラッシュ相当
* participantが存続しpublishのみ停止:ハングアップ相当

`/dev/tty*`などのデバイスファイル状態、NIC統計、カーネルログといったOS内部の情報は、DDS経由では観測できない。これらは試験終了後にSSHなどでログを回収する方式とし、試験中の実機相当PCへの介入を避ける。試験中にSSHによる監視を行う場合は、その負荷を試験条件として記録する。

### 6.5 `hils_bridge_base/reporting`

試験結果を機械可読形式で出力する。

推奨形式は以下とする。

* JSON：詳細な試験結果
* JUnit XML：CIとの統合
* CSV：時系列計測値
* Markdown：人間向けサマリ
* rosbag：ROS 2データ
* PCAP：ネットワーク試験
* バイナリダンプ：シリアル、USB、CAN試験

---

## 7. 故障モデル

### 7.1 通信消失

以下を区別する。

| 種別             | 内容             |
| -------------- | -------------- |
| Silent         | デバイスが送信を停止する   |
| No Response    | 要求に応答しない       |
| Link Down      | 通信リンク自体を切断する   |
| Device Removal | OSからデバイスを消失させる |
| Partial Loss   | 一部のメッセージのみ消失する |
| One-way Loss   | 片方向通信だけ停止する    |

単なる送信停止だけでなく、可能なインターフェースでは物理リンク切断またはOS上のデバイス消失を試験できるようにする。

### 7.2 遅延

以下の遅延モデルを実装する。

* 固定遅延
* 一様分布によるランダム遅延
* 正規分布によるジッタ
* 周期的な遅延
* バースト遅延
* 徐々に増加する遅延
* 特定メッセージだけの遅延
* 要求と応答で異なる遅延
* 一定時間の処理停止後に一括送信

### 7.3 欠損

* 一定確率で欠損
* N個ごとに欠損
* 指定したシーケンス番号を欠損
* 一定時間の連続欠損
* バースト欠損
* 特定種類のパケットだけ欠損
* パケット断片の一部欠損

### 7.4 重複と順序変更

* 同一パケットの重複
* 古いパケットの再送
* 複数パケットの順序入れ替え
* 一定時間保持した後に送信
* 再接続後に古いデータを送信

### 7.5 データ破損

故障注入は、プロトコルのどの層を破損させるか明示する。

```text
物理フレーム
トランスポート
プロトコルヘッダ
ペイロード
チェックサム
アプリケーションデータ
```

例：

* CRC不正
* チェックサム不正
* 不正なヘッダ
* 不正なマジック番号
* 不正なパケット長
* 未知のメッセージID
* 不正なデバイスID
* 不正なシーケンス番号
* 予約ビットの不正値
* 列挙値の範囲外
* NaN
* Inf
* 最大値、最小値
* 符号反転
* エンディアン不整合
* 不完全なパケット
* 途中切断されたフレーム

### 7.6 タイムスタンプ異常

* タイムスタンプ停止
* 時刻の逆行
* 大幅な未来時刻
* 大幅な過去時刻
* クロックドリフト
* 時刻の周期的なジャンプ
* UNIX時刻とデバイス時刻の混同
* 秒境界でのロールオーバー
* カウンタのオーバーフロー
* 再起動による時刻初期化

### 7.7 データ値異常

* 固定値
* ゼロ固定
* 最大値固定
* 最小値固定
* 飽和
* 突発的な外れ値
* 緩やかなバイアス増加
* スケール誤差
* 軸入れ替え
* 符号反転
* 単位誤り
* 欠損値
* 不正な状態フラグ
* 矛盾する複数フィールド

### 7.8 デバイス再起動

再起動試験では、単なる送信停止だけでなく、以下を表現する。

```text
STREAMING
    ↓
通信停止
    ↓
BOOTING
    ↓
DISCOVERABLE
    ↓
設定待ち
    ↓
READY
    ↓
STREAMING
```

設定によって以下を選択可能にする。

* 再起動後に設定を保持する
* 再起動後に初期設定へ戻る
* IPアドレスを保持する
* IPアドレスを初期化する
* シーケンス番号をゼロへ戻す
* タイムスタンプをゼロへ戻す
* ドライバからの再設定要求を必要とする
* Discoveryからやり直す
* 一定時間、設定要求を拒否する

---

## 8. 電源断試験の考え方

### 8.1 電源断の分類

本システムでは電源断を以下の3段階に分ける。

#### レベル1：論理的電源断

エミュレータがすべての送受信を停止する。

```text
状態: POWER_OFF
送信: なし
応答: なし
リンク: 維持
OSデバイス: 維持
```

実装が容易であり、ドライバのタイムアウトや上位監視の確認に使用する。

#### レベル2：通信リンク断

通信経路そのものを停止する。

例：

* Ethernetインターフェースのlink down
* 仮想Ethernetインターフェースのdown
* シリアル送受信の切断
* CANインターフェースの停止
* USBブリッジとの通信停止

OSおよびドライバがリンク異常を認識するか確認する。

#### レベル3：デバイス消失

OSからデバイスを切断する。

例：

* USBポート電源のOFF
* USBハブのポート制御
* 外部USBリレー
* USBデバイスエミュレータのdetach
* Ethernetアダプタの物理切断
* 管理可能スイッチのポート無効化

レベル3はオプション機能とし、対応ハードウェアが存在する場合のみ実施する。

### 8.2 実装上の注意

電源断機能を実装する場合でも、実センサ内部の電源回路を模擬する必要はない。

確認対象は以下とする。

* ドライバが通信断を検出するか
* ROS 2トピックが停止するか
* diagnosticsが異常へ変化するか
* 上位制御が安全側へ移行するか
* デバイス復旧後に再接続できるか
* 再接続時に設定を再送するか
* 古いデータを使用しないか
* 復旧だけで自動運転を再開しないか

---

## 9. インターフェース別実装方針

各インターフェースの故障は、PC側エミュレータのみで実現できるものと、ブリッジファームウェア(RP2040、ESP32)の協調が必要なものに分かれる。

| インターフェース | PC側で完結する故障 | ファームウェア協調が必要な故障 |
| --- | --- | --- |
| Ethernet | すべて | なし |
| シリアル(USBシリアル直結) | すべて | なし |
| USB(UVCカメラなど) | ストリーム停止、フレーム欠損、不完全フレーム | USB detach/reconnect、descriptor異常、制御転送無応答 |
| I2C/SPI | 送信データの値異常 | NACK、応答遅延、読み取り途中の値変化、レジスタ動作異常 |
| エンコーダ/PWM | 指令値の異常 | パルス停止、異常パルスパターン |

ファームウェア協調が必要な故障のために、HILSフレームプロトコル(`hils_frame_protocol.h`)へ故障注入コマンドのメッセージタイプ(0x50番台)を追加し、PC側の故障注入パイプラインからファームウェアへ故障指示を転送する。ファームウェア側は故障指示を受理した時刻と適用結果をPC側へ応答し、注入ログへ記録する。

## 9.1 Ethernet接続機器

対象例：

* LiDAR
* ネットワークカメラ
* GNSS
* Ethernet接続IMU
* 公開TCP/UDPプロトコルを持つ機器

### 実装対象

* UDP/TCPパケット
* Discovery
* ブロードキャスト
* マルチキャスト
* HTTP/REST API
* 設定コマンド
* データストリーム
* ステータス応答

### 故障注入

* パケット損失
* 遅延
* ジッタ
* 重複
* 順序変更
* 不正長
* 不正CRC
* 不正シーケンス番号
* Discoveryだけ応答停止
* 設定APIだけ応答停止
* データストリームだけ停止
* TCP接続切断
* 再接続拒否
* リンクダウン
* IPアドレス変更
* 応答元ポート変更
* 不正なHTTPステータス
* 不完全なJSON応答

### 推奨観測

* PCAP
* ソケット統計
* NIC統計
* ドライバログ
* ROS 2トピック周期
* diagnostics
* 再接続時間

---

## 9.2 シリアル接続機器

対象例：

* NMEA GPS
* バイナリIMU
* シリアルモータコントローラ
* RS-232、RS-485機器

### 実装対象

* ボーレート
* データビット
* パリティ
* ストップビット
* フロー制御
* フレーム区切り
* チェックサム
* 要求応答
* 周期送信

### 故障注入

* バイト単位の欠損
* フレーム単位の欠損
* 不完全なフレーム
* 不正チェックサム
* 不正な改行コード
* フレーム結合
* フレーム分割
* 途中停止
* 低速送信
* 一定時間の無応答
* 不正なコマンド応答
* 通信再開時の途中フレーム
* デバイスファイル消失

### 推奨観測

* 送受信バイト列
* フレーム解析結果
* ドライバ再接続回数
* `/dev/tty*`の状態
* ROS 2メッセージ周期
* diagnostics

---

## 9.3 USBデバイス

対象例：

* UVCカメラ
* USB CDC機器
* HID
* Mass Storageを利用する機器
* ベンダ固有USBクラス

### 実装対象

公開されたUSBデバイスクラス仕様または公開USBプロトコルのみを対象とする。

### 故障注入

* USBストリーム停止
* 制御転送への無応答
* 不正なdescriptor
* 不正なフレームサイズ
* 不完全な映像フレーム
* フレーム周期低下
* USB detach
* USB reconnect
* 再接続時のデバイス番号変更
* 設定変更要求の拒否
* 帯域不足相当のフレーム欠損

### 注意事項

USB descriptorの異常注入は、ホストOSやUSBコントローラの不安定化を引き起こす可能性がある。

危険なdescriptorやUSBスタックを停止させる可能性のある試験は、通常試験から分離し、専用環境で実施する。

USB detachおよびreconnectは、UVCデバイス側ファームウェアのTinyUSBが提供する`tud_disconnect()` / `tud_connect()`によりソフトウェアで実現できる。このため、電源断レベル3相当の切断試験の一部は外部リレーなしで実施可能である。

---

## 9.4 CAN接続機器

対象は、公開されているCANプロトコルに限定する。

例：

* CANopen
* J1939
* 公開されたベンダ独自CAN仕様
* 公開DBCファイルを持つ機器

### 故障注入

* CANフレーム欠損
* 不正CAN ID
* DLC不一致
* 不正データ
* 周期遅延
* heartbeat停止
* node guarding停止
* Emergency message送信
* sequence counter異常
* タイムアウト
* bus-off相当の通信停止
* 再起動後のboot-upメッセージ
* NMT状態遷移
* PDO停止
* SDO異常応答

実モータドライバ内部の電流制御やモータ相電流は対象外とする。

---

## 9.5 I2CおよびSPI

公開レジスタマップが存在する機器のみ対応する。

### 実装対象

* 識別レジスタ
* 設定レジスタ
* データレジスタ
* ステータスレジスタ
* 割り込み状態
* リセット処理
* 公開された自己診断状態

### 故障注入

* NACK
* 応答遅延
* 不正レジスタ値
* 固定値
* 読み取り途中の値変化
* リセット後の初期値
* 識別レジスタ不一致
* ステータス異常
* データ更新停止
* レジスタ読み書き拒否

NACK、応答遅延、読み取り途中の値変化など、バス上のタイミングに関わる故障はI2Cスレーブを実装するファームウェア側で実装し、PC側からは故障注入コマンドで指示する。

非公開レジスタや実センサ内部の動作は模擬しない。

---

## 10. シナリオ定義形式

故障シナリオはYAMLで記述する。

### 10.1 基本例

```yaml
scenario:
  id: lidar_power_loss_001
  description: LiDAR動作中の電源断相当試験
  target: livox_mid360
  seed: 1001

initial_state:
  device_state: streaming
  warmup_sec: 10.0

events:
  - at_sec: 20.0
    action: set_device_state
    state: power_off

  - at_sec: 30.0
    action: set_device_state
    state: booting

  - at_sec: 35.0
    action: set_device_state
    state: discoverable

expectations:
  - type: topic_timeout
    topic: /livox/lidar
    within_sec: 1.0

  - type: diagnostic_level
    node: livox_ros_driver2
    expected: error
    within_sec: 3.0

  - type: topic_resume
    topic: /livox/lidar
    within_sec: 10.0
    after_event_sec: 30.0
```

### 10.2 遅延試験

```yaml
scenario:
  id: imu_delay_ramp_001
  target: witmotion_wt901

events:
  - from_sec: 10.0
    to_sec: 40.0
    action: delay_ramp
    start_delay_ms: 0
    end_delay_ms: 500

expectations:
  - type: maximum_message_age
    topic: /imu/data
    threshold_ms: 200

  - type: diagnostic_transition
    expected: warn_or_error
    within_sec: 2.0
```

### 10.3 不正データ試験

```yaml
scenario:
  id: gps_checksum_error_001
  target: gps_nmea0183

events:
  - from_sec: 15.0
    to_sec: 25.0
    action: corrupt
    field: checksum
    probability: 0.5

expectations:
  - type: invalid_message_not_published
    topic: /fix

  - type: process_alive
    node: nmea_serial_driver
    expected: true
```

---

## 11. 故障注入API

各デバイスエミュレータは、共通の制御APIを持つことが望ましい。

ROS 2サービスまたはactionとして、以下を定義する。

```text
/set_device_state
/inject_fault
/clear_fault
/load_scenario
/start_scenario
/stop_scenario
/get_scenario_state
/get_device_state
/get_fault_state
```

### 11.1 故障注入要求の概念

```yaml
fault_type: delay
target: data_stream
parameters:
  delay_ms: 250
  jitter_ms: 30
duration_sec: 10
```

### 11.2 状態参照

以下を取得可能にする。

* 現在のデバイス状態
* 有効な故障
* シナリオ進捗
* 送信パケット数
* 欠損パケット数
* 破損パケット数
* 平均遅延
* 最大遅延
* 最終通信時刻
* 最終要求時刻
* 最終応答時刻

---

## 12. 試験判定

故障を注入するだけでなく、期待される動作を自動判定する。

### 12.1 判定対象

* ROS 2ノードがクラッシュしない
* 異常データを正常値として公開しない
* 一定時間内に異常を検出する
* diagnosticsが所定レベルになる
* 古いデータを継続使用しない
* 指令出力を停止する
* 安全側の状態へ遷移する
* 通信復旧後に再接続する
* 再接続後に設定をやり直す
* 古いパケットを破棄する
* 復旧だけで自動運転を再開しない
* 手動リセットまたは明示的な再開操作を要求する

### 12.2 合否判定例

```yaml
expectations:
  - id: EXP-001
    condition: driver_process_alive
    expected: true

  - id: EXP-002
    condition: diagnostic_error
    within_ms: 2000

  - id: EXP-003
    condition: command_output_zero
    within_ms: 500

  - id: EXP-004
    condition: automatic_restart
    expected: false
```

---

## 13. Watchdog検証方針

本システムで確認するwatchdogは、公開された通信境界より上位に存在するものとする。

対象例：

* ROS 2ノード間heartbeat
* ドライバデータ更新監視
* `cmd_vel`更新監視
* CANopen heartbeat
* 公開プロトコル上のalive counter
* 外部安全PLCへ送信する運転許可信号
* モータドライバの公開command timeout

### 13.1 注入する故障

* heartbeat停止
* 同じheartbeatの繰り返し
* counter停止
* counter逆行
* timestamp停止
* データ更新だけ停止
* heartbeatだけ継続
* 制御指令だけ継続
* センサだけ停止
* 応答遅延
* 通信断
* デバイス再起動

### 13.2 確認項目

* 所定時間内に異常を検出する
* 異常の種類を識別できる
* 出力が安全側へ移行する
* エラーがラッチされる
* 復旧のみで自動再始動しない
* 明示的なリセットが必要である
* イベント時刻が記録される

---

## 14. 安全関連機能との境界

本システムは、安全機器そのものとして使用しない。

Raspberry Pi Pico、ESP32、通常のROS 2ノードおよびHILS Bridgeは、以下の用途に限定する。

* 故障注入
* 信号生成
* 信号計測
* 試験シナリオ制御
* ログ収集
* 合否判定支援

非常停止、安全PLC、STO、安全リレーなどを試験する場合は、実際の安全機器を試験対象として接続する。

HILS装置側が安全機能を代替してはならない。

安全関連試験では、少なくとも以下を区別する。

```text
試験対象の安全機能
試験装置の制御機能
試験装置自身を安全に停止する機能
```

故障注入装置が異常動作した場合でも、試験対象設備が危険な動作をしない構成とする。

---

## 15. 実装フェーズ

## Phase 1：共通故障注入基盤

実装内容：

* 既存エミュレータの共通基盤への移行
  * `UdpEmulatorBase` / `SerialBridgeBase`への送信フック(故障パイプライン差し込み点)の追加
  * Livoxエミュレータの`UdpEmulatorBase`への移行
  * カメラノードの共通シリアル基盤への移行と、重複した`frame_protocol.py`の削除
* `hils_bringup`へのシナリオ・launch基盤の整備
* 故障注入基底クラス
* 故障パイプライン
* イベントスケジューラ
* YAMLシナリオローダ
* ログ出力
* 乱数シード管理
* ROS 2制御API

既存エミュレータは送信処理を直接呼び出しており、故障パイプラインの差し込み点が存在しない。共通基盤への移行を最初に行うことで、故障注入無効時の正常系互換性(17.4節)を回帰確認しながら開発を進める。

対象故障：

* delay
* jitter
* drop
* duplicate
* corrupt
* freeze

## Phase 2：通信断と再起動

実装内容：

* デバイス状態機械
* logical power off
* communication disconnect
* reboot
* startup delay
* configuration reset
* reconnect

## Phase 3：Ethernet機器への適用

優先対象：

1. Livox Mid-360
2. Velodyne VLP-16
3. Ouster OS1

理由：

* パケット単位の故障注入を実装しやすい
* PCAPによる確認が容易
* 実ドライバが存在する
* Discovery、設定、ストリームを分離して試験できる

## Phase 4：シリアル機器への適用

優先対象：

1. NMEA 0183 GPS
2. Witmotion WT901

実装内容：

* バイト単位故障
* フレーム単位故障
* チェックサム異常
* 不完全フレーム
* シリアル切断
* 再接続

## Phase 5：USB機器への適用

優先対象：

* UVCカメラ
* USB CDC機器

実装内容：

* HILSフレームプロトコルへの故障注入コマンド(0x50番台)追加
* ファームウェア側故障注入(TinyUSBによるdetach/reconnectなど)
* ストリーム停止
* フレーム欠損
* 不完全フレーム
* 再接続
* 設定要求異常

## Phase 6：試験オラクルとCI

実装内容：

* 期待条件の自動評価
* JUnit XML出力
* GitHub ActionsまたはローカルCIとの統合
* nightly試験
* 実機相当PCを使用した定期試験
* テストレポート生成

---

## 16. 優先実装項目

最初のリリースでは、以下を優先する。

### 必須

* YAMLによるシナリオ定義
* 固定遅延
* ランダムジッタ
* パケット欠損
* 不正データ
* 送信停止
* デバイス再起動状態
* ROS 2サービスによる開始・停止
* イベントログ
* シードによる再現性
* 正常系への復帰
* 既存エミュレータへの影響を最小化

### 推奨

* シナリオの自動合否判定
* rosbag自動記録
* PCAP自動記録
* JUnit XML出力
* diagnostics監視
* ノード死活監視
* トピック周期監視
* メッセージ鮮度監視

### 将来対応

* USB物理切断
* 管理スイッチによるEthernetポート制御
* 外部リレーによる電源制御
* CAN bus-off試験
* 安全PLC用I/O試験装置
* Web UIによるシナリオ編集
* 複数デバイス同時故障
* 統計的故障注入
* 長時間耐久試験

---

## 17. 非機能要件

### 17.1 再現性

同じ入力、同じ設定、同じシードで同じ故障系列を生成できること。

### 17.2 観測可能性

すべての故障注入について、以下を記録する。

* 予定時刻
* 実行時刻
* 対象
* 故障種別
* パラメータ
* 適用結果
* 解除時刻

### 17.3 拡張性

新しいデバイスを追加する際、故障注入共通基盤を再実装しないこと。

### 17.4 正常系互換性

故障注入を無効にした場合、既存エミュレータの正常動作を変更しないこと。

### 17.5 フェイルセーフ

故障注入プロセスが停止した場合、意図しないデータを送り続けないこと。

設定により以下を選択できるようにする。

* 送信停止
* 正常系へ戻る
* 接続を閉じる
* デバイス状態をPOWER_OFFへ移行する

既定動作は送信停止または接続切断とする。

### 17.6 時刻精度

故障注入時刻の許容誤差を定義する。

一般的なROS 2ドライバ試験では、初期目標を以下とする。

```text
イベント開始誤差: ±10 ms以内
イベント終了誤差: ±10 ms以内
ログ時刻分解能: 1 ms以下
```

より高い精度が必要なインターフェースでは、マイコン側タイマまたはハードウェア計測を使用する。

### 17.7 遅延注入の実装戦略

遅延およびジッタの注入は、送信処理とは独立した遅延キュー(優先度付きキューと専用スレッド)で実装する。送信予定時刻の早い順にパケットを取り出して送出することで、遅延と順序入れ替えを同一機構で扱う。

PythonのGILとOSスケジューリングにより1〜2ms程度の揺らぎが残るため、注入精度は仕様値ではなく実測で示す。Ethernet系はPCAPのタイムスタンプ、シリアル系は受信側のバイト到着時刻により、目標精度を満たすことを確認してから試験に使用する。

統計的な遅延・ジッタのみが必要な場合は、4.5節のnetemバックエンドを使用することで、Python側の時刻精度の制約を回避できる。

### 17.8 複数PC間の時刻同期

シミュレーションPCと実機相当PCのイベントログを突き合わせるため、chronyまたはPTPによる時刻同期を必須とする。

試験結果には同期方式と同期状態(オフセット推定値)を記録する。時刻同期が確立していない状態での試験結果は、PC間の時刻比較を含む判定に使用しない。

---

## 18. コーディング方針

* デバイス固有コードと故障注入コードを分離する
* 正常パケット生成後に故障パイプラインを適用する
* プロトコルフィールド単位の故障はデバイス固有層で実装する
* バイト列単位の故障は共通層で実装する
* Transport層の故障は通信アダプタで実装する
* すべてのパラメータに範囲チェックを行う
* 不正なシナリオは試験開始前に拒否する
* 実行中の例外を握りつぶさない
* 故障注入の適用回数を統計情報として公開する
* ROS 2 parameterで変更可能な設定と、試験開始後に変更禁止の設定を区別する
* デバイス固有仕様の参照元をコードまたは文書に記録する
* プロトコルバージョンを明示する
* 実ドライバの対応バージョンを記録する

---

## 19. ドキュメント構成案

```text
docs/
├── fault_injection/
│   ├── overview.md
│   ├── architecture.md
│   ├── fault_model.md
│   ├── scenario_format.md
│   ├── observation_and_oracle.md
│   ├── ethernet_faults.md
│   ├── serial_faults.md
│   ├── usb_faults.md
│   ├── can_faults.md
│   └── safety_boundary.md
│
├── scenarios/
│   ├── lidar/
│   ├── camera/
│   ├── gps/
│   ├── imu/
│   └── actuator/
│
└── test_reports/
```

各デバイスの文書には以下を記載する。

* 対象機器
* 公開仕様
* 対応プロトコルバージョン
* 対応ドライバ
* 正常系の動作
* 対応する故障
* 対応しない故障
* 状態遷移
* 接続構成
* 実行方法
* 合否判定
* 既知の制約

---

## 20. 完了条件

初期実装は、少なくとも1種類のEthernet LiDARと1種類のシリアルセンサについて、以下を満たした時点で完了とする。

1. 正常系データを実ドライバが受信できる
2. YAMLシナリオを読み込める
3. 送信停止を注入できる
4. 固定遅延を注入できる
5. ランダム欠損を注入できる
6. 不正チェックサムまたは不正パケットを注入できる
7. デバイス再起動相当の状態遷移を実行できる
8. 故障解除後に正常系へ復帰できる
9. 注入イベントをログへ記録できる
10. 同一シードで再現できる
11. rosbagまたはPCAPを保存できる
12. ドライバがクラッシュしないことを確認できる
13. トピック停止またはdiagnostics変化を自動判定できる
14. テスト結果をJUnit XMLまたはJSONで出力できる
15. 故障注入無効時に既存動作へ影響しない

---

## 21. まとめ

本システムでは、実センサ内部や実モータドライバ内部の非公開プロトコルを模擬しない。

代わりに、公開された機器プロトコルおよび物理通信インターフェースを利用し、実際のROS 2ドライバと実機相当PCを試験対象とする。

重点を置くのは以下である。

* 正常データの互換性確認
* 電源断相当の通信消失
* 通信遅延
* データ欠損
* 不正データ
* デバイス再起動
* 設定異常
* 通信復旧
* watchdog
* diagnostics
* 上位アプリケーションの安全側遷移

この方針により、非公開仕様の解析に伴う高い開発コストを避けながら、SILSでは十分に確認できない実ドライバ、OS、物理通信経路、タイムアウト、再接続、異常処理を検証できるHILS基盤を構築する。

---

## 22. 実装状況と確認事項

### 22.1 実装状況(2026-07-29時点)

Phase 1(共通故障注入基盤)、Phase 2(通信断と再起動)、Phase 3(Ethernet機器への適用)、Phase 4(シリアル機器への適用、ソフトウェアで到達可能な範囲)、およびPhase 6の初期実装(試験オラクル)は実装済み。

Phase 1:

* 故障パイプライン:delay(ジッタ含む)、drop、corrupt、duplicate、freeze、reorder
* シードによる故障系列の再現(ユニットテストで検証済み)
* ROS 2制御API:`~/inject_fault`、`~/clear_fault`、`~/get_fault_state`
* YAMLシナリオ:loader/validator、イベントスケジューラ、`scenario_runner`ノード
  (ループバック実測で注入時刻誤差 0.3〜0.5 ms)
* 共通基盤への移行:Velodyne、Livox、UVCカメラ(GPS/WT901は`serial_write`経由で自動対応)
* `hils_bringup`:Velodyne用シナリオ例とlaunch

Phase 2:

* デバイス状態機械(4.2節の状態集合、状態別チャネルゲート、抑止パケット数の記録)
* `~/set_device_state`、`~/get_device_state`サービス
* 論理的電源断(8.1節レベル1):`power_off`で全チャネル停止
* 再起動(7.8節):疑似状態`reboot`で`rebooting`→`boot_duration_sec`後に
  `reboot_target_state`へ自動遷移
* シナリオアクション`set_device_state`
* Livoxの状態連動:電源断・再起動でWorkModeをリセットし、ドライバによる
  再設定(WorkModeControl再送)を要求。再起動後は`discoverable`へ復帰
* 電源断中は要求パケットの解析自体を停止(実デバイス同様に無応答)

Phase 3:

* 対象3機種(Livox Mid-360、Velodyne VLP-16、Ouster OS1)すべてが
  共通故障注入基盤とデバイス状態機械に接続済み
* チャネル別故障:Livoxはdiscovery/command/data/imu、Velodyneは
  data/position、Ousterはdata/imu/httpを個別に故障対象化できる
  (9.1節「Discoveryだけ応答停止」「設定APIだけ応答停止」に対応)
* Ouster HTTP API故障:`http_status`故障による不正HTTPステータス、
  corrupt(truncate)による不完全JSON応答、dropによる無応答、
  delayによる低速応答。電源断状態ではTCP応答なし
* エミュレータ試験用に`http_port`パラメータを追加(実機互換の既定値80)

Phase 4:

* フィールド認識故障:`nmea_checksum`(NMEA文のチェックサムのみ不正化、
  文自体は整形式のまま)、`wt901_checksum`(WT901 11バイトフレームの
  和チェックサムのみ不正化)。バイト単位・フレーム単位の故障
  (drop/corrupt/truncate/freeze/reorder)は共通層がserial_write経由で
  既に適用可能
* デバイス固有故障の登録フック`register_fault_class()`
* socatのPTYペアによりFT234Xハードウェアなしで実ドライバ
  (`nmea_navsat_driver`)へのE2Eを実施:チェックサム不正注入で
  `/fix`が即停止(ドライバは100件のチェックサムエラーを記録しつつ
  クラッシュせず)、故障解除の0.13秒後に復帰。オラクルの3期待条件
  すべてPASS(12.1節「異常データを正常値として公開しない」を実証)
* FT234X実機経路での回帰(2026-07-29実施):FT234X×2クロス接続
  (`/dev/ttyUSB0`↔`/dev/ttyUSB1`、`docker/docker-compose.serial.yml`で
  simコンテナへパススルー)により、USBシリアルのOSスタック(FTDI
  ドライバ、実ボーレート、バッファリング)経由で同一シナリオを回帰。
  `run_gps_e2e.sh`/`run_imu_e2e.sh`は`E2E_SERIAL_BRIDGE`/
  `E2E_SERIAL_DRIVER`環境変数で実ポートに切り替えられる
* GPS(9600 baud):PTY版と同一の3件PASS(停止t=10.00s、復帰t=20.11s、
  ドライバ生存)。懸念していたボーレート起因のフレーム分割による
  判定差は発生しなかった
* WT901実ドライバE2E(115200 baud):`witmotion_ros`(ElettraSciComp版、
  ソースビルド)に対し`wt901_checksum`故障を注入する
  `scenarios/imu/wt901_checksum_error_001.yaml`と`tools/run_imu_e2e.sh`を
  追加。PTY・実ポートの両方で3件PASS(停止t=10.06s、復帰t=20.06s、
  ドライバ生存)。`witmotion_ros`はパケット検証有効
  (`ValidatePackets(true)`)のためチェックサム不正フレームを破棄し、
  12.1節「異常データを正常値として公開しない」をWT901系でも実証

Phase 5(USBファームウェア協調故障、UVC分。2026-07-31実施):

* プロトコル:フレームプロトコルに0x50番台メッセージを追加
  (`0x50` FAULT_SET、`0x51` FAULT_CLEAR、`0x52` FAULT_ACK(逆方向)、
  `0x5F` RESET_BOOTSEL(マジック保護))。定義はC/Python両側で共有
  (`hils_frame_protocol.h`/`frame_protocol.py`)
* PC側:`FirmwareFault`基底+`uvc_usb_detach`/`uvc_frame_drop`/
  `uvc_partial_frame`。`FaultPipeline`に`on_added`/`on_removed`フックと
  ファームウェア転送(`set_firmware_transport()`、注入・解除・期限切れで
  自動送信。パイプライン迂回のためdrop故障が自分のclearを飲み込まない)
  を追加。クラスは`hils_bridge_base`配置でシナリオランナーから検証可能。
  ファーム無しブリッジ(GPS等)では注入時に明確なエラーで拒否
* ファームウェア側(`rp2040_camera_uvc`):`fault_handler.c`が
  detach(`tud_disconnect`+期限付き再接続)、frame_drop(決定論的間引き、
  乱数不使用)、partial_frame(送信段で切り詰め)を実行し、全コマンドに
  ACKを返す
* 実機確認(Picoペア):
  - `uvc_usb_detach`(reconnect_after_ms=5000):注入直後にホストから
    デバイス消失(`lsusb`/`/dev/video0`とも)、5秒後に自動再列挙
  - `uvc_frame_drop`(100%、6秒):ホスト側で115フレーム連続同一
    (約5.3秒フリーズ)→解除後回復
  - `uvc_partial_frame`(keep 50%、6秒):**切り詰められたJPEGが
    UVCホストへ実際に到達**(6秒間で242/256フレームがEOI欠落)。
    ホスト側の寛容なデコーダでは上部のみ描画された画像になる。
    22.2節のとおりホスト側byte故障ではPico#1が棄却するため、
    壊れJPEG系の試験はこのファームウェア故障で行うこと
  - `RESET_BOOTSEL`:リモートでBOOTSELへ再起動→UF2コピー→再列挙の
    ラウンドトリップを確認(以後Pico#2の物理BOOTSEL操作は不要)
* I2C(MPU-6050)側(2026-07-31実施。Pico×2+4.7kΩプルアップ×2、
  スレーブGPIO3を3.3Vレールとして使用):
  - スレーブ(`rp2040_imu_invensense_mpu6050`)に4故障を実装:
    `i2c_nack`(IC_ENABLE=0でバスから消失)、`i2c_response_delay`
    (トランザクション先頭バイトのクロックストレッチ、arg0=µs)、
    `i2c_reg_freeze`(シミュレーション更新停止=固定値)、
    `i2c_who_am_i`(識別レジスタ不一致、arg0=返す値)。
    0x50番台コマンド・ACK・RESET_BOOTSELはUVCと同一機構を再利用
  - 実ドライバ代役として`rp2040_mpu6050_reader`(マスター役Pico)を
    新規作成:WHO_AM_I確認→wake→100Hzバースト読み(0x3B-0x48)+
    周期WHO再確認(10秒毎)を行い、全結果をCDCテキストで報告
    (`[STAT]`/`[WHO]`/`[ERR]`/`[REINIT]`行。ホスト側判定に使える)
  - 実機E2E(全PASS):正常系はaz=16384(1g)・gz=3753(0.5rad/s)の
    理論値完全一致。nack 4秒=+378 NACK後に自動回復、
    delay 15ms=全てtimeoutに正しく分類(+298)、
    reg_freeze 8秒=固定中は入力変更を無視し解除後に追従、
    who_am_i=故障窓内の周期確認で0x71 ok=0→解除後0x68 ok=1
  - 発見:RP2040のI2Cマスターは、スレーブ消失でタイムアウトした後
    コントローラが固着し以後の全トランザクションがtimeoutになる。
    リーダーにバスリカバリ(SCL 9クロック+deinit/init再初期化)を
    実装して解消。RP2040ベースの実制御器を被試験対象にする際に
    同じ罠を踏む可能性がある点は試験観点として有用

Phase 6(初期実装):

* `scenario_oracle`ノード:1プロセス内に2つのrclpyコンテキストを持ち、
  制御側(simドメイン)でシナリオランナーの進捗・イベント実績時刻を取得、
  観測側(robotドメイン)へ読み取り専用で参加してトピック到着時刻と
  ノードグラフを記録する(6.4節の観測配置をそのまま実装)。
  観測購読はrawモードでペイロードを復号しないため点群でも軽量
* 対応する期待条件:`topic_timeout`、`topic_resume`、`topic_alive`、
  `node_alive`/`process_alive`(DDS participantの存在で代理判定)、
  `maximum_message_age`(header.stampと到着時刻の差。rosbag再生の
  古いstampでは無意味な点に注意)、`diagnostic_level`(/diagnostics
  のレベル遷移)。未対応タイプはFAILではなくSKIPとして報告
* レポート:JSON(詳細)とJUnit XML(CI用)を出力し、終了コード0/1で
  合否を返す
* docker E2E検証:Livox再起動シナリオを実ドライバに対して自動判定
  (停止t=10.00s、復帰t=17.07sを検出しPASS)。負例として
  `reboot_config_policy:=reset`では復帰なしを正しくFAIL判定
  (22.3節のSDK2制約が機械判定可能な回帰テストになった)
* rosbag自動記録(`record_bag`。故障前ベースラインから記録)、PCAP記録
  (`record_pcap`。tcpdumpとCAP_NET_RAWが必要、無ければ警告のみ)
* CI:`.github/workflows/hils_tests.yml`(ビルド+単体テスト+GPS E2Eを
  push/PR/nightlyで実行)。ハードウェア不要E2Eは`tools/run_gps_e2e.sh`
* 未対応(今後):`invalid_message_not_published`、netemバックエンド

未実施作業の引継ぎ手順は[fault_injection_handover.md](fault_injection_handover.md)を参照。

### 22.2 未確認事項

実装変更後の回帰確認が未完了の項目は現在ない(残る未実施作業は
ハードウェア導入待ちのもの。引継ぎ資料3.3節以降を参照)。

解消済み:

* UVCホスト互換性の改善(2026-07-31):`usb_cam`(0.8.1)で映像取得
  できなかった2要因を解消し、`pixel_format:=mjpeg2rgb`で15Hz取得・
  解像度逆コマンド・drop故障のフリーズ/回復を実機確認した。
  - UVCディスクリプタのフレームインターバルをcontinuous(Stepwise)から
    **Discrete(15/10/5fps×3解像度)へ変更**(`bFrameIntervalType=3`)。
    従来のStepwise宣言はTinyUSBサンプル踏襲で意図した設計ではなく、
    `usb_cam`はDISCRETE列挙しか扱えないため起動すらできなかった。
    実カメラの列挙もほぼDiscreteであり忠実度も向上(要Pico#2再書き込み)
  - ブリッジのJPEGエンコードを**4:2:2クロマサブサンプリングへ変更**
    (PIL使用。OpenCV 4.6の`imencode`は4:2:0固定)。`usb_cam`の
    `mjpeg2rgb`はswscaleコンテキストを4:2:2前提で事前構築するため、
    4:2:0フレームでは色差プレーンのバッファ外読みでsegfaultする
    (実UVCカメラのMJPEGは4:2:2が通例で、実機では顕在化しない)。
    `raw_mjpeg`(デコードなし)では4:2:0でも取得できることから
    切り分けた
* UVCカメラ実機経路の回帰(2026-07-30):Picoペア
  (`rp2040_cdc_spi_sender`+`rp2040_uvc_bridge`)経由でSerialBridgeBase
  移行後の全経路を実機確認。
  - 正常系:`cam2image`→ブリッジ→Pico#1→UART→Pico#2→UVCで
    `/dev/video0`からMJPGを取得、1345フレーム全デコード成功。
    解像度逆コマンドも3解像度(320x240/640x480/1280x720)で
    STREAMONの度にブリッジの`frame_width/height`パラメータへ即時反映
  - 故障注入のUVCホスト側挙動:**3故障ともUVCストリーム自体は
    停止せず「映像フリーズ」として現れる**(Pico#2は新フレームが
    来ない間、最終フレームをUVCフレームレートで再送し続けるため)。
    drop(5秒)=4.3秒フリーズ後に回復、corrupt(bit_flip)=dropと同一の
    フリーズ(Pico#1がフレームXORチェックサム検証で壊れたフレームを
    棄却するため、**壊れJPEGはホストへ一切届かない**。ホスト側
    デコード不能フレーム0/1345。12.1節をファームウェア層で実証)、
    delay(1000ms)=注入直後に約1.1秒フリーズ後、遅延した映像が流れる
  - 発見1:Pico#1に別ファームウェア(サーボPWM計測)が載っている
    状態では、CDCを読まない相手へのブロッキング書き込みで
    ブリッジnodeのexecutorが無限停止し、パラメータ/サービス応答も
    死ぬ。対策としてSerialBridgeBaseに`write_timeout_sec`(既定1.0s)を
    追加し、タイムアウトした書き込みはエラーログを出して破棄する
    ようにした(単体92件PASS維持)
  - 発見2:pico-sdkの1200baudタッチが有効なため、BOOTSELボタン
    なしで`ros2 run`環境からファームウェア書き換えが可能
    (1200baudでopen→close→RPI-RP2マウント→uf2コピー)
  - 補足:故障回復直後の7/1345フレームにEOI後の余剰バイトを観測
    (全て正常デコード可能。UVCフレーム境界のバッファ残留とみられる)
* FT234X実機経路のGPS/WT901回帰(2026-07-29):22.1節Phase 4のとおり
  実ポートでPTY版と同一の判定(各3件PASS)を確認。付随する発見:
  `witmotion_ros`の既定`timeout_ms: 150`は「ポートオープン後150ms以内に
  データ着信」を要求するため、ブリッジ起動とのレースでSUSPENDED状態に
  落ち、その後のQtタイマー跨ぎスレッド操作でsegfaultする(実機WT901は
  電源投入直後からストリームするため顕在化しない)。E2Eスクリプトでは
  設定パッチで`timeout_ms: 2000`に拡大して回避。このタイムアウトは
  完全無音でのみ発火し、チェックサム故障中もバイトは流れ続けるため
  試験判定には影響しない
* Ouster実ドライバ疎通(2026-07-29):`ouster_ros` 0.14.1(apt)で
  HTTPメタデータ取得→UDP受信の正常系を確認。`/ouster/points`約5Hz
  (ドライバのスキャン組み立て後)、IMU約187Hz、壁10mがレンジ分布
  p90=10.7mに出現。`http`チャネルへのdrop注入中も点群ストリームは
  継続し(設定APIのみ無応答)、解除後にHTTP 200へ復帰することを
  実ドライバに対して確認
* `hils_bridge_encoder_quadrature`ビルド失敗:`config/default_params.yaml`
  を追加して解消(全12パッケージのビルド成功)

### 22.3 Livox実ドライバ回帰(2026-07-29実施、解消済み)

docker構成(`hils_sim`/`hils_robot`)で、MID-360実機のrosbag
(`~/git_ws/3D_LIDAR_TEST/ros_ws/exp1_bags`、壁距離2/5/10/15/20mの5本、
PointCloud2+Imu)をシミュレーションデータ源として`livox_ros_driver2`
1.2.6に対する回帰を実施した。

正常系(壁10m bag、20秒計測):

| 指標 | bag元データ | ドライバ再発行 |
| --- | --- | --- |
| 点群レート | 9.74 Hz | 10.05 Hz |
| レンジ中央値 | 3.86 m | 3.86 m(完全一致) |
| レンジ90パーセンタイル | 9.39 m | 9.35 m |
| IMU | 200 Hz | 受信良好 |

Discovery→設定→WorkMode Normal→点群/IMU受信のSDK2ハンドシェイク一式、
および電源断→再起動→自動復帰(14.2秒ギャップ後にドライバ無操作で再開)
を確認した。

#### 発見事項:Livox SDK2は既知デバイスを再設定しない

故障注入により以下のドライバ側特性が判明した。

* SDK2はLidarSearchブロードキャストを1秒周期で送信し続けるが、
  一度設定が完了したデバイス(既知handle)に対しては、Detection応答を
  受けても再設定(WorkModeControl再送)を行わない
  (`GeneralCommandHandler::HandleDetectionData`の`is_update_cfg`は
  初期設定完了後にリセットされない)
* コマンドポートへのハートビートは存在せず、SDKはデバイスの消失を
  検出しない(60秒の無応答でも切断扱いにならない)
* したがって実機MID-360が電源断から復帰できるのは、デバイス側が
  WorkMode設定を保持し自律的にストリーミングを再開するためである

これを受け、エミュレータに`reboot_config_policy`パラメータを追加した。

* `preserve`(既定):実機同様、再起動後にWorkModeを復元して自動再開
* `reset`:再起動でWorkModeを喪失。SDKの上記制約により、ドライバを
  再起動するまでストリームは復旧しない(ドライバの再接続性欠陥を
  再現する故障シナリオとして使用可能)

### 22.4 パラメータ名の変更(移行時の互換性)

| 対象 | 旧 | 新 | launch引数 |
| --- | --- | --- | --- |
| Livoxノード | `lidar_ip` | `device_ip` | `lidar_ip`のまま(内部でマッピング) |
| カメラノード | `max_fps` | `max_hz` | `max_fps`のまま(内部でマッピング) |
