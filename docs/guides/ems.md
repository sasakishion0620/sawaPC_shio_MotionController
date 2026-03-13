# EMS（電気筋肉刺激）電圧制御ガイド

ロボットが物を掴んだときの力（`f_dis`）に応じて、EMS 回路に 0〜3.3V を自動出力する機能の説明。

## 概要

Bilateral モードでは、ロボットハンドの反力推定値（`f_dis`）から EMS 電圧を自動計算し、DA ボードの ch1 から出力する。操作者は掴む力に応じた電気刺激フィードバックを受ける。

他のモード（idle / remote 等）では、GUI から手動で ch1 電圧を入力できる。

## 使い方

1. `sudo ./control` で起動
2. GUI 下部のモードボタンで **Bilateral** を選択
3. EMS 電圧が自動計算され、DA ch1 から出力される
4. GUI に現在の EMS 電圧と f_dis(0) がリアルタイム表示される

他のモードに切り替えると、GUI に電圧入力欄（0〜3.3V）が表示され、手動で ch1 電圧を設定できる。

## パラメータ調整

`config/ems.json` を編集する:

```json
{
  "force_threshold": 0.5,
  "force_max": 20.0,
  "voltage_threshold": 0.5,
  "voltage_max": 3.3
}
```

| パラメータ          | 意味                                       |
| ------------------- | ------------------------------------------ |
| `force_threshold`   | この力以下では EMS 出力 0V（ノイズ除去用） |
| `force_max`         | この力で最大電圧に達する                   |
| `voltage_threshold` | EMS 回路が反応し始める最低電圧 (V_th)      |
| `voltage_max`       | EMS 回路の最大電圧                         |

### 変換式

```
f_dis < force_threshold の場合:
    v_ems = 0

それ以外:
    v_ems = V_th + (V_max - V_th) / (force_max - force_threshold) * (f_dis - force_threshold)
    v_ems = clamp(v_ems, 0, V_max)
```

グラフで見ると:

```
力 (f_dis)
 0.0 ── 0.5 ──────────────── 20.0 ──→
         │                     │
         ▼                     ▼
電圧    0V      0.5V ─────── 3.3V
         ↑
    ここまでは 0V
   （閾値以下）
```

### 調整の目安

- **刺激が弱すぎる** → `voltage_threshold` を上げる、または `force_max` を下げる
- **軽く触れただけで刺激される** → `force_threshold` を上げる（例: 1.0〜2.0）
- **強く掴んでも刺激が変わらない** → `force_max` を上げる

## データフロー

```
Bilateral モード:
  エンコーダ → DOB → f_dis(0) → EMS 電圧計算 → dict["da_ch1_voltage"] → DA ch1 → EMS 回路
                                              → dict["ems_voltage"]    → GUI 表示

その他のモード:
  GUI 手動入力 → dict["da_ch1_voltage"] → DA ch1 → EMS 回路
```

## コードの場所

EMS に関係するコードは 4 箇所:

### 1. EMS 電圧計算（変換式を変えたい場合はここ）

`src/controller.cc` — Bilateral コントローラ内の `// EMS voltage calculation from f_dis(0)` ブロック:

```cpp
double v_ems = 0.0;
if (f_dis(0) >= f_threshold)
{
    v_ems = v_th + (v_max - v_th) / denom * (f_dis(0) - f_threshold);
}
// → dict["da_ch1_voltage"] に書き込み
// → dict["ems_voltage"] に書き込み（GUI表示用）
```

例えば線形変換を S 字カーブに変えたい場合、この計算部分だけ変更すればよい。

### 2. パラメータ読み込み

`inc/system_controller.h` — `load_ems_config()`:

`config/ems.json` を読み、dict に以下のキーで格納:
- `ems_force_threshold`
- `ems_force_max`
- `ems_voltage_threshold`
- `ems_voltage_max`

### 3. GUI 表示

`inc/gui_widget.h` — `da_voltage_control()`:

- Bilateral モード時: EMS 電圧と f_dis(0) を Text 表示（読み取り専用）
- その他モード時: InputFloat で手動入力（0〜3.3V）

### 4. DA ボード出力

`inc/contec_da.h` — `write_buf()`:

dict の `"da_ch1_voltage"` を毎サイクル読み取り、DA ボードの ch1 に出力する。EMS 側のコードを変更する必要はない。

## dict キー一覧

コードから EMS の値を参照する場合:

| キー                    | 内容                                | 読み書き |
| ----------------------- | ----------------------------------- | -------- |
| `da_ch1_voltage`        | DA ch1 に出力される電圧値 [V]       | R/W      |
| `ems_voltage`           | 同上のコピー（GUI 表示用）          | R/W      |
| `ems_force_threshold`   | config から読み込んだ force 閾値     | R        |
| `ems_force_max`         | config から読み込んだ force 最大値   | R        |
| `ems_voltage_threshold` | config から読み込んだ voltage 閾値   | R        |
| `ems_voltage_max`       | config から読み込んだ voltage 最大値 | R        |

任意の場所で `robot.get_from_dict("da_ch1_voltage")` のように取得できる。
