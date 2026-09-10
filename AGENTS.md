# AGENTS.md — led_matrix_firmware

## プロジェクト概要
- RP2040 + Arduino (Earle Philhower core) + 74HC595 LEDマトリクス用ファームウェア。
- プロトコルv2: 固定4Bヘッダ＋RAWペイロード＋COBS＋`0x00`区切り。`src/protocol.h` が正。
- ビルド: `pio run -e waveshare_rp2040_zero` → `.pio/build/waveshare_rp2040_zero/firmware.uf2`
- マルチコア: Core0=受信・展開、Core1=表示。V-Syncダブルバッファ（ポインタflipのみ）。

## 駆動タイミング（フリーラン・固定）— 編集時はこの仕様を維持すること
- **データ有無に関わらず駆動**: Core1 `loop1()` (`src/main.cpp:250-283`) は `frame_ready` 待ちせず常時16行スキャン。起動時は `memset(pio_stream, 0)` の消灯スキャン（行選択のみ、`SIN_2/3=0`）。受信データの反映はBAMサイクル境界での `stream_front` 切替のみ。
- **精度・速度は画素値に関わらず固定**: シフト時間＝DMA 64word/行＋PIO固定クロック（clkdiv 2.5＝50MHz、`src/shift_out.pio:74`）。点灯時間＝プレーン番号のみで決定：`BAM_UNIT_US(2us) << plane` から `STROBE_OH_US(3us)` を差し引き下限 `BAM_MIN_ON_US(2us)` (`src/main.cpp:273-276`)。画素の0/1で `delayMicroseconds` は変えないこと。
- **ビット数とHz**: `N=1〜8` はフレーム単位で可変（`PROTO_MAX_BITS 8`、`src/protocol.h:10`）。HzはNで一意に決まる：1bit 約150Hz（`BINARY_ON_US 400us`×16行）、6bit 約370Hz、8bit 約110Hz。初期値は1bit。
- **固定しない方針**: ビット深度を固定化しないこと。可変でもモード毎にタイミングは決定的で、1bit＝輝度スケール迂回の最大輝度モードと多階調の使い分けが有益。固定化するなら `6bit/370Hz` を選ぶ（8bitはLSB 1〜2usでOH補正限界のため精度低下）。

## 注意
- READMEの6bit表記は旧記述。コード（1〜8bit）を正とする。
- Core0診断printは `dbg_can_write()` ガード必須（TX枯渇でCore0停止するため）。
- LATCHはPIO駆動（SET pin）。GPIOとして `pinMode` しないこと。
