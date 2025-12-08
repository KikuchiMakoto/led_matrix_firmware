# LED Matrix Firmware for RP2040

RP2040専用のLEDマトリクスファームウェアです。74HC595シフトレジスタを使用したLEDマトリクス表示に対応し、8bit輝度制御をサポートしています。

## 特徴

- **8bit輝度対応**: Bit-Angle Modulation (BAM)による256段階の輝度制御
- **高速動作**: PIO（Programmable I/O）を使用した高速シフト出力
- **互換性**: 従来の1bitモードとの後方互換性を維持
- **マルチコア**: RP2040の2コアを活用（Core0: データ受信、Core1: 表示更新）
- **USB CDC-ACM**: Base64エンコードされたデータをUSB経由で受信

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

## 通信プロトコル

### データフォーマット
USB CDC-ACM経由でBase64エンコードされたデータを送信します。各フレームは改行文字（`\n`）で終了します。

### 動作モード

#### 1bitモード（互換モード）
- **データサイズ**: 256 bytes（Base64デコード後）
- **用途**: 従来のON/OFF表示（互換性維持）
- **動作**: 各ビットは0x0000または0xFFFFに展開され、全ビットプレーンで表示

#### 8bitモード（輝度対応）
- **データサイズ**: 2048 bytes（Base64デコード後）
- **用途**: 256段階の輝度制御
- **データ構造**: 8つのビットプレーン × 256 bytes
  - 各ビットプレーンは元のmatrix_buffer[8][16]形式（uint16_t）

### エラー処理
- 256 bytes、2048 bytes以外のデータは無視されます
- Base64デコードエラー時もデータは無視されます

## ビルド方法

### 必要なもの
- CMake 3.13以上
- GCC ARM Embedded Toolchain
- Git

### リポジトリのクローン

```bash
# リポジトリをクローン
git clone <repository-url>
cd LED_Matrix_firmware_K00798

# サブモジュール（Pico SDK）を初期化
git submodule update --init --recursive
```

### ビルド

```bash
# ビルドディレクトリを作成
mkdir build
cd build

# CMake実行とビルド
cmake ..
make -j4

# 生成されたファイル: led_matrix_firmware.uf2
```

### 書き込み

```bash
# RP2040をBOOTSELモードで接続
# （BOOTSELボタンを押しながらUSB接続）

# .uf2ファイルをRP2040のマスストレージにコピー
cp led_matrix_firmware.uf2 /media/$USER/RPI-RP2/
```

## 使用方法

1. RP2040ボードをBOOTSELモードで接続（BOOTSELボタンを押しながらUSB接続）
2. `.uf2`ファイルをRP2040のマスストレージにコピー
3. 自動的に再起動し、ファームウェアが動作開始
4. USB CDC-ACMデバイスとして認識されます
5. データを送信（例: Pythonスクリプト）

### 送信例（Python）

```python
import serial
import base64

# デバイスを開く
ser = serial.Serial('/dev/ttyACM0', 921600, timeout=1)

# 1bitモードの例（256 bytes）
data_1bit = bytearray(256)
# データを設定...

encoded = base64.b64encode(data_1bit)
ser.write(encoded + b'\n')

# 8bitモードの例（2048 bytes）
data_8bit = bytearray(2048)
# 8つのビットプレーンデータを設定...

encoded = base64.b64encode(data_8bit)
ser.write(encoded + b'\n')

ser.close()
```

## データ構造の詳細

### matrix_buffer構造
```
matrix_buffer[8][16] (as uint16_t)
  ├─ [0][0-15]: Panel 3, SIN_3
  ├─ [1][0-15]: Panel 3, SIN_2
  ├─ [2][0-15]: Panel 2, SIN_3
  ├─ [3][0-15]: Panel 2, SIN_2
  ├─ [4][0-15]: Panel 1, SIN_3
  ├─ [5][0-15]: Panel 1, SIN_2
  ├─ [6][0-15]: Panel 0, SIN_3
  └─ [7][0-15]: Panel 0, SIN_2
```

### 8bitモードのビットプレーン
- ビットプレーン0（LSB）: 最も弱い輝度ビット
- ビットプレーン7（MSB）: 最も強い輝度ビット
- BAMにより、各ビットプレーンは重み付けされた時間で表示（2^n）

## ライセンス

オリジナルのArduino版との互換性を保ちつつ、RP2040専用に最適化されています。

## 参考

- ハードウェア仕様: https://cba.sakura.ne.jp/kit01/kit_464.htm
- Raspberry Pi Pico SDK: https://github.com/raspberrypi/pico-sdk
