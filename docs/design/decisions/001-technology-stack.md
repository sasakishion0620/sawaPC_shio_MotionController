<!--
種別: decisions
対象: 技術スタック選定
作成日: 2026-02-26
更新日: 2026-02-26
担当: AIエージェント
-->

# 技術スタック選定

## 概要

teleop-hand の技術スタックを選定する。ロボットハンドのリアルタイム制御と finger-tracker との通信を実現するための言語・ライブラリ・ハードウェアインターフェースを決定する。

## 設計判断

### 判断1: 言語 — C++

**問題**: メイン開発言語を何にするか

**選択肢**:
1. C++
2. Python
3. C++ + Python ハイブリッド

**決定**: C++

**理由**:
- controller_try が C++ で書かれており、移植・改修のベースとなる
- リアルタイム制御ループに必要な低レイテンシ・決定的実行時間を確保できる
- Contec PCI ボードのI/Oアクセス（`iopl`, `outl`, `inl`）が C/C++ で直接可能

**トレードオフ**:
- **利点**: リアルタイム性能、ハードウェアアクセス、controller_try との互換性
- **欠点**: 開発速度は Python に劣る、finger-tracker（Python）との通信にプロセス間通信が必要

### 判断2: ビルドシステム — CMake

**問題**: ビルドシステムを何にするか

**選択肢**:
1. CMake
2. Makefile
3. Meson

**決定**: CMake

**理由**:
- controller_try が CMake を使用
- ライブラリ依存関係の管理が容易
- IDE サポートが充実

**トレードオフ**:
- **利点**: controller_try との互換性、広く使われているため情報が豊富
- **欠点**: CMakeLists.txt の記述が冗長になりやすい

### 判断3: 線形代数 — Eigen

**問題**: 行列演算ライブラリを何にするか

**選択肢**:
1. Eigen
2. Armadillo
3. 自前実装

**決定**: Eigen

**理由**:
- controller_try で既に使用
- ヘッダオンリーで導入が容易
- ロボティクス分野で広く使われている

**トレードオフ**:
- **利点**: 高性能、ヘッダオンリー、広いコミュニティ
- **欠点**: コンパイル時間が長くなりやすい

### 判断4: GUI — OpenGL + GLFW + GLEW

**問題**: GUI フレームワークを何にするか

**選択肢**:
1. OpenGL + GLFW + GLEW（controller_try 踏襲）
2. Qt
3. Dear ImGui

**決定**: OpenGL + GLFW + GLEW（controller_try のGUI構造を踏襲）

**理由**:
- controller_try の GUI コードを改修して使用するため
- ライブ描画に適している（制御状態をリアルタイムに表示）

**備考**: GUI スレッドは制御タイマーに同期しない非リアルタイムスレッド（OS 描画速度に依存、30〜60fps 程度）。制御ループ（10kHz）とは独立して動作する。

**トレードオフ**:
- **利点**: controller_try との互換性、ライブ描画性能
- **欠点**: 高レベルなUI部品がなく、ウィジェット実装が手作業

### 判断5: ハードウェアI/O — Contec PCI ボード

**問題**: モータ制御のハードウェアインターフェースを何にするか

**選択肢**:
1. Contec PCI DA/カウンタボード（既存）
2. EtherCAT
3. USB DAQ

**決定**: Contec PCI DA/カウンタボード

**理由**:
- 研究室の既存ハードウェア資産
- controller_try でドライバコードが実装済み
- PCI バス経由の低レイテンシI/O

**トレードオフ**:
- **利点**: 既存資産活用、低レイテンシ、実績あるドライバコード
- **欠点**: PCI スロット搭載PCが必要、root権限が必要

### 判断6: 通信方式 — UDP ソケット

**問題**: finger-tracker（Python）と teleop-hand（C++）間の通信方式を何にするか

**決定**: UDP ソケット（localhost）

**理由**: 詳細は [ADR 002](./002-communication-protocol.md) を参照

### 判断7: 設定ファイル — JSON (Boost.PropertyTree)

**問題**: 設定ファイルの形式と読み込みライブラリを何にするか

**選択肢**:
1. JSON (Boost.PropertyTree)
2. YAML (yaml-cpp)
3. TOML

**決定**: JSON (Boost.PropertyTree)

**理由**:
- controller_try が JSON + Boost.PropertyTree を使用
- 関節パラメータの JSON ファイルがそのまま再利用可能

**トレードオフ**:
- **利点**: controller_try の設定ファイルとの互換性、追加依存なし
- **欠点**: JSON はコメントが書けない（必要ならキー名で代用、controller_try と同様）

## 技術スタック一覧

| レイヤー | 技術 | 用途 |
|---------|------|------|
| 言語 | C++ | メイン言語 |
| ビルド | CMake | ビルドシステム |
| 線形代数 | Eigen | 行列演算 |
| GUI | OpenGL + GLFW + GLEW | リアルタイムGUI |
| ハードウェアI/O | Contec PCI DA/カウンタ | モータ制御・エンコーダ |
| 設定 | JSON (Boost.PropertyTree) | 設定ファイル読み込み |
| 通信 | UDP ソケット (localhost) | finger-tracker 連携（[ADR 002](./002-communication-protocol.md)） |

## 関連ドキュメント

- [002-communication-protocol.md](./002-communication-protocol.md) — 通信方式の詳細（UDP ソケット）
- [003-robot-hand-specification.md](./003-robot-hand-specification.md) — ロボットハンド・モータ仕様
- [004-control-architecture.md](./004-control-architecture.md) — 制御アーキテクチャ
- [005-control-philosophy.md](./005-control-philosophy.md) — 制御思想
- [初期仕様書](../../archive/initial_plan.md)
- [CLAUDE.md](../../../CLAUDE.md)
