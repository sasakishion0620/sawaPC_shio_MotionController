<!--
種別: status
対象: 実装ステータス
作成日: 2026-02-26
更新日: 2026-02-26
担当: AIエージェント
-->

# 実装ステータス

## モジュール別ステータス

| モジュール | ステータス | 備考 |
|-----------|----------|------|
| comm | `not-started` | finger-tracker との通信 |
| controller | `not-started` | 制御ロジック |
| hardware | `not-started` | ハードウェアI/O |
| config | `not-started` | 設定管理 |
| gui | `not-started` | GUI表示 |
| scripts | `not-started` | ユーティリティ |

## 機能別ステータス

| 機能 | モジュール | ステータス | 備考 |
|------|-----------|----------|------|
| 指令値受信 | comm | `not-started` | UDP ソケット（[ADR 002](../design/decisions/002-communication-protocol.md)） |
| 制御モード切替 | controller | `not-started` | controller_try から移植 |
| モータ制御 | controller, hardware | `not-started` | controller_try から移植 |
| DA出力 | hardware | `not-started` | controller_try から移植 |
| エンコーダ読取 | hardware | `not-started` | controller_try から移植 |
| 関節パラメータ読込 | config | `not-started` | controller_try から移植 |
| GUI表示 | gui | `not-started` | controller_try から移植 |
