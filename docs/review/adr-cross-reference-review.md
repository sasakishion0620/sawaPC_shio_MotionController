<!--
種別: review
対象: ADR相互参照・整合性レビュー
作成日: 2026-02-26
更新日: 2026-02-26
担当: AIエージェント
-->

# docs (相互参照・整合性) レビュー結果

## Critical

- **[C-1]** ADR 001 の技術スタック一覧表で「通信」の技術が「未定」のまま残っている
  - 該当箇所: `docs/design/decisions/001-technology-stack.md:155`
  - 理由: 判断6（行 118-124）でUDPソケットに決定し、ADR 002 が詳細を定義しているにもかかわらず、技術スタック一覧（`| 通信 | 未定 | finger-tracker 連携 |`）が更新されていない。CLAUDE.md の技術スタック表（行 63）では `UDP ソケット (POSIX socket)` と正しく記載されており、矛盾が生じている
  - 修正案: ADR 001 の技術スタック一覧を下記に更新する
    ```markdown
    | 通信 | UDP ソケット (POSIX socket) | finger-tracker 連携（[ADR 002](./002-communication-protocol.md)） |
    ```

- **[C-2]** ADR 002 の「関連ドキュメント」セクションのコメントが不正確（「通信方式を『未定』とした判断」という記述）
  - 該当箇所: `docs/design/decisions/002-communication-protocol.md:152`
  - 理由: ADR 001 の判断6（行 118-124）では「詳細は ADR 002 を参照」と書かれており、通信方式の判断は ADR 001 の時点で「UDP ソケット」に決定している。ADR 002 を「通信方式を『未定』とした判断」と参照するのは誤りであり、ADR 001 の実際の内容と矛盾する
  - 修正案: 下記のように修正する
    ```markdown
    - [001-technology-stack.md](./001-technology-stack.md) — 通信方式（UDP ソケット）の判断。詳細はこのADRを参照する旨が記載されている
    ```

- **[C-3]** implementation.md の「指令値受信」機能ステータス備考が「通信方式は今後決定」のまま古い状態で残っている
  - 該当箇所: `docs/status/implementation.md:26`
  - 理由: ADR 002 でUDPソケットに決定済みであるにもかかわらず、implementation.md には「通信方式は今後決定」と記載されている。ステータス文書がADRの決定内容と一致しない
  - 修正案: 備考を下記のように修正する
    ```markdown
    | 指令値受信 | comm | `not-started` | UDP ソケット（[ADR 002](../design/decisions/002-communication-protocol.md)） |
    ```

## Medium

- **[M-1]** ADR 004 の `recv_command` スレッドの設計が「非同期（データ到着時）」とされているが、ADR 002 のノンブロッキング受信実装と設計の整合性が不明瞭
  - 該当箇所: `docs/design/decisions/004-control-architecture.md:192-198`
  - 理由: ADR 004 の判断5（スレッド表）では `recv_command` を「非同期（データ到着時）」として独立スレッドと記述しているが、`recv_command` の設計説明（行 196-203）では「ノンブロッキングUDP受信（`recvfrom` + `MSG_DONTWAIT`）」と記述している。これら2つは矛盾しないが、「ノンブロッキング受信のために別スレッドが必要か」という疑問が残る。ADR 002 の C++ 受信コード（行 134-139）でも `MSG_DONTWAIT` を使っており、これは制御ループ内から直接呼ぶ設計として記述されている
  - 修正案: ADR 004 の `recv_command` スレッドについて、「制御ループ内に組み込む（`compute_engine` か `read_sensor` から呼ぶ）か、独立スレッドにするか」を明示的に決定し、ADR 002 のコード例との整合を取る。例：
    ```markdown
    **`recv_command` の設計**:
    - 制御ループ（`read_sensor` または `compute_engine`）の先頭でノンブロッキング UDP 受信 (`recvfrom` + `MSG_DONTWAIT`) を呼び出す方式とする
    - 独立スレッドではなく制御ループ内に統合することで、スレッド安全性の問題を排除する
    ```

- **[M-2]** ADR 004 の制御則実装例（行 131）では `θ_cmd_filtered` を用いていないが、同ADR内の信号処理パイプライン（行 94）では `θ_cmd_filtered` をステップ 6.5 に含めている
  - 該当箇所: `docs/design/decisions/004-control-architecture.md:131` および `94`
  - 理由: 信号処理パイプラインでは「6.5. 指令値フィルタ: θ_cmd_filtered = LPF(θ_cmd_raw, g_cmd)（ADR 005）」とフィルタ適用が明記されているが、直後に示す制御則の実装コード（行 131）では `θ_cmd` と記述されており、フィルタ後の変数名 `θ_cmd_filtered` と一致しない
  - 修正案: ADR 004 の制御則コードを下記に修正する
    ```cpp
    f_ref = M * (k_p * (θ_cmd_filtered - θ_res) + k_v * (0 - dθ_res))
    ```
    （ADR 005 行 100 の記述と一致させる）

- **[M-3]** ADR 003 の「関連ドキュメント」に ADR 004 と ADR 005 への参照が欠落している
  - 該当箇所: `docs/design/decisions/003-robot-hand-specification.md:138-142`
  - 理由: ADR 003 で定義するリンク長（r = 0.0725 m）は ADR 004（行 119）および ADR 005（行 29）の逆運動学の計算式で直接使用されている。しかしADR 003 の「関連ドキュメント」には ADR 004 と ADR 005 への参照がない。ADR 004（行 208-211）と ADR 005（行 186）から ADR 003 への参照は存在するが、逆方向の参照が欠けている
  - 修正案: ADR 003 の「関連ドキュメント」セクションに以下を追加する
    ```markdown
    - [004-control-architecture.md](./004-control-architecture.md) — 制御ループ・逆運動学の利用（リンク長パラメータを参照）
    - [005-control-philosophy.md](./005-control-philosophy.md) — 余弦定理による逆運動学（リンク長 r = 0.0725 m を使用）
    ```

- **[M-4]** ADR 001 の「関連ドキュメント」に ADR 002-005 への参照が欠落している
  - 該当箇所: `docs/design/decisions/001-technology-stack.md:157-160`
  - 理由: ADR 001 は技術スタックの基盤を定義しており、ADR 002-005 はそれを前提として参照しているが、ADR 001 自身の「関連ドキュメント」には ADR 002-005 への参照が全くない。特に「判断6」で「詳細は ADR 002 を参照」と本文で言及しているにもかかわらず、「関連ドキュメント」セクションには ADR 002 が含まれていない
  - 修正案: ADR 001 の「関連ドキュメント」セクションに以下を追加する
    ```markdown
    - [002-communication-protocol.md](./002-communication-protocol.md) — 通信方式の詳細（判断6の参照先）
    - [003-robot-hand-specification.md](./003-robot-hand-specification.md) — ハードウェア仕様の詳細
    - [004-control-architecture.md](./004-control-architecture.md) — 制御アーキテクチャの詳細
    - [005-control-philosophy.md](./005-control-philosophy.md) — 制御思想の詳細
    ```

## Low

- **[L-1]** ADR 002 の設定表（行 144-148）では `finger-tracker の config.yaml` とあるが、ADR 001（行 143）では JSON + Boost.PropertyTree を設定形式として決定しており、設定ファイル形式が異なっている
  - 該当箇所: `docs/design/decisions/002-communication-protocol.md:146`
  - 提案: teleop-hand 側は JSON（ADR 001 の決定による）であり、finger-tracker 側の `config.yaml` は finger-tracker プロジェクトの設定形式であることを明記すると混乱が減る
    ```markdown
    | 送信先 IP | finger-tracker の `config.yaml`（finger-tracker プロジェクトの設定） | `127.0.0.1` |
    | ポート番号 | 両プロジェクトの設定ファイル（teleop-hand 側は JSON） | `50000` |
    ```

- **[L-2]** ADR 003 の判断4にある構成図（行 98-112）が `remote` モードの通信として「指間距離」のみ矢印表示しているが、ADR 002 のパケット仕様では `distance_mm + red_pos[3] + blue_pos[3]` の 7 float（28 bytes）を送信する
  - 該当箇所: `docs/design/decisions/003-robot-hand-specification.md:107`
  - 提案: 構成図の矢印ラベルを `指間距離 + 3D座標` と修正するか、注釈として「実際のパケット仕様は [ADR 002](./002-communication-protocol.md) を参照」を追記する

- **[L-3]** ADR 005 の判断3（行 114）での問題設定において制御ループを「10kHz」と述べているが、ADR 004 の判断4（行 150-178）では「設定ファイルで変更可能（デフォルト 10kHz）」と定義しており、ハードコードされた周波数の記述である
  - 該当箇所: `docs/design/decisions/005-control-philosophy.md:118`
  - 提案: ADR 005 の記述を「10kHz（デフォルト設定値、[ADR 004](./004-control-architecture.md) を参照）」と記述することで、設定変更可能な点を明示する

- **[L-4]** ADR 004 と ADR 005 の逆運動学の式の表記が微妙に異なっている
  - 該当箇所: `docs/design/decisions/004-control-architecture.md:128` および `docs/design/decisions/005-control-philosophy.md:44`
  - 提案: ADR 004（行 128）では `θ_cmd = cos⁻¹(1 - x_d² / (2r²))` と一行で表記しており、ADR 005（行 43-44）では `cos⁻¹((x_d² - 2r²) / (2r²))` と導出形式も示している。同一の式であり数学的に等価であるが、同じ最終形式 `cos⁻¹(1 - x_d² / (2r²))` に統一するとより明確になる

- **[L-5]** roadmap.md の Phase 1 タスクに `record_motion` スレッドへの言及がない
  - 該当箇所: `docs/status/roadmap.md:16-20`
  - 提案: ADR 004 の判断5（スレッド構造、行 187-203）では `record_motion`（データ記録、10kHz）スレッドが定義されているが、roadmap.md の Phase 1 タスク一覧に記録機能が含まれていない。スコープ外ならその旨を補足すると明確になる

## 確認済み

- ADR 001-005 のメタデータ形式（HTMLコメント: 種別・対象・作成日・更新日・担当）が全て統一されている
- ADR 001-005 のセクション構成（概要・設計判断・関連ドキュメント）が全て統一されている
- ADR 001-005 の各判断内の形式（問題・選択肢・決定・理由・トレードオフ・不採用理由）が統一されている
- CLAUDE.md の技術スタック表の言語・ビルド・線形代数・GUI・ハードウェアI/O・JSON・通信の各行が ADR 001・ADR 002 の決定内容と一致している
- CLAUDE.md の処理フロー図（行 82-95）に記載の制御モード（idle / DA_check / remote / Bilateral）が ADR 004 の判断1の決定と一致している
- CLAUDE.md の制御ループ記述（行 98-105）の信号処理順（擬似微分→DOB→制御則→トルクリミット→電圧変換）が ADR 004 の信号処理パイプラインと一致している
- ADR 002 のポート番号（50000）が CLAUDE.md の処理フロー（行 86）の `localhost:50000` と一致している
- ADR 003 のエンコーダ仕様（6750 P/R、4逓倍で 27000 P/R）と ADR 003 の関節パラメータ表（pulse_per_rotation: 6750.0、multiplication: 4.0）が一致している
- ADR 003 のギア減速比（gear_ratio: 3.0）と判断3の「ギア減速比 3:1」が一致している
- ADR 004 の `compute_engine` における `sampling_frequency: 10`（100kHz / 10 = 10kHz）の計算が正しく、ADR 004 の判断4の説明と一致している
- ADR 004 → ADR 005、ADR 005 → ADR 004 の相互参照（双方向リンク）が存在している
- ADR 005 の制御則コード（行 100）の変数名 `θ_cmd_filtered` が ADR 004 の信号処理パイプライン（行 94）の記述と一致している（M-2 は ADR 004 の実装例のみの問題）
- roadmap.md の Phase 2 で「通信方式の決定」がタスクとして残っているが、これは ADR 002 が作成された現時点では達成済みの判断であり、Phase 2 の実装作業として整合する
- CLAUDE.md の用語表（行 109-119）に定義されている DOB・RFOB・EMS・バイラテラル制御・DA ボード等の用語が、各 ADR での用語使用と一致している
