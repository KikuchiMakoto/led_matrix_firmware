# LED Matrix Firmware for RP2040 (Arduino)

RP2040専用のLEDマトリクスファームウェアです。Arduinoフレームワーク（Earle Philhower版 arduino-pico）ベースで、74HC595シフトレジスタを使用したLEDマトリクス表示に対応し、最大8bit輝度制御をサポートしています。

## 特徴

- **最大8bit輝度対応**: Bit-Angle Modulation (BAM)による最大256段階の輝度制御（1〜8bit可変）
- **PIO高速シフト**: PIOステートマシンによるハードウェアシフト出力＋STROBEブランキングでジッター排除
- **COBSバイナリ通信**: Base64を廃止し、固定4Bヘッダ＋COBS＋0x00区切り（オーバーヘッド約0.5%）
- **マルチコア**: RP2040の2コアを活用（Core0: データ受信、Core1: 表示更新、V-Syncダブルバッファ）
- **USB CDC-ACM**: 1200bps touchでUF2書き込みモードへ自動遷移（BOOTSELボタン不要）

## 転送エンジン

- PIOステートマシンによるシフト出力（SIN_1/SIN_2/SIN_3＋CLOCK sideset）
- PIOクロックは125MHz / 2.5 = 50MHz起点（約12.5MHzピクセルレート、`src/shift_out.pio`のclkdivで調整）
- シフト中はSTROBE=HIGH（ブランキング）でLSB埋没を防止

## 駆動タイミング（フリーラン・固定）

- **データ有無に関わらず駆動**: Core1 (`loop1`) は受信待ちせず常時16行スキャン。起動直後はゼロクリア済みバッファの消灯スキャン（行選択のみ走査、`SIN_2/3=0`）。受信フレームはV-Sync境界でのポインタflipのみで反映。
- **精度・速度は画素値に関わらず固定**: シフト時間はDMA 64word/行＋PIO固定クロック、点灯時間はプレーン番号のみで決定（`BAM_UNIT_US(2us) << plane` から `STROBE_OH_US(3us)` を差し引き、下限 `BAM_MIN_ON_US(2us)`）。画素の0/1で `delayMicroseconds` は変わらない。
- **ビット数とHz**: ビット深度 `N=1〜8` はフレーム単位で可変、リフレッシュはNで一意に決まる。実測値: 1bit 約150Hz（`BINARY_ON_US 400us`×16行）、6bit 約370Hz、8bit 約110Hz。切替はBAMサイクル境界でのみ行うためグリッチなし。
- **固定しない理由**: 可変でもタイミングはモード毎に決定的で、1bit＝輝度スケール迂回の最大輝度モードと、多階調6〜8bitを使い分けできる柔軟性の方が有益なため。固定するなら `6bit/370Hz` 推奨（8bitはLSB 1〜2usでオーバーヘッド補正限界のため精度が落ちる）。

## ハードウェア

### 対応ボード
- Waveshare RP2040-Zero（または互換RP2040ボード）
- 74HC595ベースのLEDマトリクスボード

### ピン配置
| ピン | 機能 | 説明 |
|------|------|------|
| GPIO 0 | SIN_1 | 行選択データ |
| GPIO 1 | SIN_2 | パネルデータ1 |
| GPIO 2 | SIN_3 | パネルデータ2 |
| GPIO 3 | CLOCK | シフトクロック |
| GPIO 4 | LATCH | ラッチ信号 |
| GPIO 5 | STROBE | ストローブ信号 |

## 通信プロトコル v2（COBS、Base64廃止）

### データフォーマット
USB CDC-ACM経由でバイナリフレームを送信します。各フレームは `COBS(ヘッダ4B＋ペイロード)＋0x00` です。

```
[0] MAGIC = 0x55
[1] MODE  = ビット深度 N (0x01〜0x08)
[2..3] PAYLOAD_LEN = N*256 (uint16 LE)
[4..] ペイロード: Nプレーン × 256B（各プレーンはmatrix_buffer[8][16] uint16LE、plane0=LSB先頭）
```

### 動作モード

#### 1bitモード（N=1、ペイロード256B）
- **用途**: ON/OFF表示（Dashboardモードの第一目標）
- 固定点灯時間で表示

#### グレースケールモード（N=2〜8、ペイロード512〜2048B）
- **用途**: 4〜256段階の輝度制御、BAM表示
- 推奨: 60fpsは6bit、120fpsは4〜6bit

### エラー処理
- MAGIC/MODE/LENの3点照合に失敗したパケットは破棄されます
- 次の `0x00` で即再同期します

## ビルド方法（PlatformIO＋Arduino）

### 必要なもの
- Python 3.11以上
- PlatformIO (`pip install platformio`)

### ビルド

```bash
pio run -e waveshare_rp2040_zero
```

生成されたファイル: `.pio/build/waveshare_rp2040_zero/firmware.uf2`

### 書き込み

```bash
# 方法A: 1200bps touchで自動的にUF2モードへ（Arduinoファーム動作中に有効）
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0',1200); s.close()"

# 方法B: BOOTSELボタンを押しながらUSB接続

# RPI-RP2ドライブにコピー（自動再起動）
cp .pio/build/waveshare_rp2040_zero/firmware.uf2 /media/$USER/RPI-RP2/
```

## 使用方法

1. `.uf2`ファイルをRP2040のマスストレージ（RPI-RP2）にコピー
2. 自動的に再起動し、ファームウェアが動作開始
3. USB CDC-ACMデバイス（`/dev/ttyACM0`）として認識されます
4. Pythonから送信（例: led_matrix_software）

### 送信例（Python）

```python
from led_matrix_software.devices import SerialLEDDevice
from led_matrix_software.matrix import make_matrix_buffer

dev = SerialLEDDevice('/dev/ttyACM0')
dev.write(make_matrix_buffer(img))          # 1bit (N=1)
dev.write_grayscale(gray16x128, bits=6)     # 6bit grayscale
dev.close()
```

## データ構造の詳細

### matrix_buffer構造
```
matrix_buffer[8][16] (as uint16_t)
  ├─ [0][0-15]: Panel 0, SIN_2
  ├─ [1][0-15]: Panel 0, SIN_3
  ├─ [2][0-15]: Panel 1, SIN_2
  ├─ [3][0-15]: Panel 1, SIN_3
  ├─ [4][0-15]: Panel 2, SIN_2
  ├─ [5][0-15]: Panel 2, SIN_3
  ├─ [6][0-15]: Panel 3, SIN_2
  └─ [7][0-15]: Panel 3, SIN_3
```
（注: シフト先頭がチェーン奥に届くため、先頭シフトのPanel 3が右端グループ(6,7)を担う。実機検証済み）

### グレースケールのビットプレーン
- ビットプレーン0（LSB）: 最も弱い輝度ビット
- ビットプレーンN-1（MSB）: 最も強い輝度ビット（N≦8）
- BAMにより、各ビットプレーンは重み付けされた時間で表示（2^n × BAM_UNIT_US、現在2us）

## 開発メモ：安定版v2とv3の試行結果

安定版は**v2即時モード**（固定4Bヘッダ＋COBS＋`0x00`区切り、受信即V-Syncスワップ）です。実測リフレッシュ：8bit 約110Hz、6bit 約370Hz。

v3として以下を試行しましたが、実機で安定せずv2に戻しました（コードは `fw-v3-queue` ブランチに退避）：

- 5Bヘッダ（`CLASS/DEPTH`上位ニブル＋FPS要求バイト0〜255、0＝即時）
- ファームウェア側パケットキュー（深さ8、フル時 oldest-drop）＋ペース提示スケジューラ
- Flow制御報告（`+HB`に`q=`/`drop=`追加）とホスト側リーダースレッド

症状：複数プレーン（4bit以上）フレーム受信後にCore0が停止し、USBが無応答化。4秒watchdogで自動リブートするも再現性100%で実用に至らず。1bitフレームは正常処理されるため、デコード〜展開の大サイズ経路との相関が濃厚。原因は未特定（TinyUSBヒープ枯渇・DMA競合・受信リング境界などが候補）。再挑戦時は単発8bit受信→`av=`/`rx=`/`ok=`カウンタ監視の切り分け手順が有効でした。

## ライセンス

オリジナルのArduino版との互換性を保ちつつ、RP2040専用に最適化されています。

## 参考

- ハードウェア仕様: https://cba.sakura.ne.jp/kit01/kit_464.htm
- Raspberry Pi Pico SDK: https://github.com/raspberrypi/pico-sdk
