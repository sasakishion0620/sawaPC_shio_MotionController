<!--
種別: decisions
対象: 通信方式選定
作成日: 2026-02-26
更新日: 2026-02-26
担当: AIエージェント
-->

# 通信方式選定

## 概要

finger-tracker（Python）と teleop-hand（C++）間のプロセス間通信方式を選定する。同一 Linux PC 上で動作する 2 プロセス間で、指間距離と 3D 座標をリアルタイムに伝送する。

## 設計判断

### 判断1: 通信プロトコル — UDP ソケット

**問題**: 異なる言語（Python / C++）で書かれた 2 プロセス間で、リアルタイム制御データを伝送する方式を何にするか

**選択肢**:
1. UDP ソケット
2. TCP ソケット
3. 共有メモリ (POSIX shm)
4. ZeroMQ (Pub/Sub)

**決定**: UDP ソケット（localhost, ポート番号は設定ファイルで指定）

**理由**:
- Python / C++ 両方で標準ライブラリのみで実装可能（外部依存なし）
- コネクションレスのため、finger-tracker と teleop-hand の起動順序に依存しない
- 同一マシンの localhost 通信ではパケットロスがほぼ発生しない
- 将来的に別 PC へ分離する場合も、IP アドレスを変更するだけで対応可能
- 実装がシンプルで、デバッグも容易（Wireshark, tcpdump 等で監視可能）

**トレードオフ**:
- **利点**: シンプル、低レイテンシ、外部依存なし、起動順序非依存、将来の PC 分離に対応
- **欠点**: 信頼性保証なし（パケットロスの可能性）。ただし localhost では実質ロスなし。カメラ FPS（30Hz 程度）で送信するため、1 パケット欠落しても 33ms 後に次のデータが届く

**不採用理由**:
- **TCP**: コネクション確立が必要で起動順序に依存する。リアルタイム制御では head-of-line blocking による遅延リスクがある
- **共有メモリ**: 最低レイテンシだが、排他制御の実装が複雑。同一 PC 専用で将来の PC 分離に対応できない
- **ZeroMQ**: 柔軟だが、外部ライブラリ依存が増える。現段階では 1:1 通信のみなので Pub/Sub の利点が活きない

### 判断2: ペイロード形式 — バイナリ（リトルエンディアン float 配列）

**問題**: UDP パケットのデータ形式を何にするか

**選択肢**:
1. バイナリ（float 配列、固定長）
2. JSON 文字列
3. Protocol Buffers

**決定**: バイナリ（リトルエンディアン float 配列、固定長 28 bytes）

**理由**:
- パースのオーバーヘッドがゼロ（C++ 側で `memcpy` のみ）
- Python 側は `struct.pack` で簡単に生成可能
- データ構造が固定で単純なため、スキーマ定義は不要

**トレードオフ**:
- **利点**: 最小のパースコスト、最小のペイロードサイズ、実装が簡単
- **欠点**: 可読性がない（デバッグ時は別途ツールが必要）、フィールド追加時はバージョニングが必要

**不採用理由**:
- **JSON**: 人間可読だがパースコストが高い。リアルタイム制御ループ内での使用に不適
- **Protocol Buffers**: スキーマ管理が堅牢だが、7 個の float を送るだけの用途には過剰

### 判断3: 送信データ — 指間距離 + 各指の 3D 座標

**問題**: finger-tracker から teleop-hand へ何のデータを送信するか

**選択肢**:
1. `distance_mm` のみ（float 1 個 = 4 bytes）
2. `distance_mm` + `red_pos[3]` + `blue_pos[3]`（float 7 個 = 28 bytes）
3. 上記 + `red_conf` + `blue_conf` + `timestamp`（float 10 個 = 40 bytes）

**決定**: `distance_mm` + `red_pos[3]` + `blue_pos[3]`（float 7 個 = 28 bytes）

**理由**:
- 指間距離だけでは、将来のバイラテラル制御（個別の指位置が必要）に対応できない
- 3D 座標を送ることで、単純な距離変換以外の制御戦略にも対応可能
- 28 bytes は UDP の最小パケットサイズ内に十分収まる

**トレードオフ**:
- **利点**: 将来のバイラテラル制御に対応、ペイロードが十分小さい
- **欠点**: distance_mm のみの場合より 7 倍大きい（ただし絶対値として 28 bytes は無視できるサイズ）

## パケット仕様

```
オフセット  サイズ  フィールド       説明
0           4       distance_mm     指間距離 (mm), float32 LE
4           4       red_x           赤指 X 座標 (m), float32 LE
8           4       red_y           赤指 Y 座標 (m), float32 LE
12          4       red_z           赤指 Z 座標 (m), float32 LE
16          4       blue_x          青指 X 座標 (m), float32 LE
20          4       blue_y          青指 Y 座標 (m), float32 LE
24          4       blue_z          青指 Z 座標 (m), float32 LE
────────────────────────────────────────────────────────────
合計        28 bytes
```

- エンディアン: リトルエンディアン（x86 ネイティブ）
- 検出失敗時: `distance_mm = -1.0` を送信（受信側で無効データとして処理）

### Python 送信側（finger-tracker）

```python
import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ("127.0.0.1", 50000)  # ポート番号は config で指定

# distance_mm, red_pos[3], blue_pos[3]
packet = struct.pack("<7f", distance_mm, *red_pos, *blue_pos)
sock.sendto(packet, target)
```

### C++ 受信側（teleop-hand）

```cpp
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>

struct finger_data {
    float distance_mm;
    float red_x, red_y, red_z;
    float blue_x, blue_y, blue_z;
};

// 受信（recv_command スレッド内でノンブロッキング受信、ADR 004 参照）
finger_data data;
ssize_t n = recvfrom(sock_fd, &data, sizeof(data), MSG_DONTWAIT, nullptr, nullptr);
if (n == sizeof(data) && data.distance_mm >= 0.0f) {
    // 有効データ
}
```

## 設定

| パラメータ | 場所 | デフォルト |
|-----------|------|-----------|
| 送信先 IP | finger-tracker の `config.yaml` | `127.0.0.1` |
| ポート番号 | 両プロジェクトの設定ファイル | `50000` |
| 送信レート | finger-tracker のカメラ FPS | 30 Hz |

## 関連ドキュメント

- [001-technology-stack.md](./001-technology-stack.md) — 技術スタック選定
- [004-control-architecture.md](./004-control-architecture.md) — recv_command スレッド設計
- [初期仕様書](../../archive/initial_plan.md)
