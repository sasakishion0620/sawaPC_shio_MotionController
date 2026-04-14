# GitHub 送信 / Linux 取得 手順メモ

このメモは、以下をまとめたものです。

- Windows 側で変更を GitHub に送る方法
- Linux 側でこのプロジェクトをコピーする方法
- VS Code で作業するときの流れ

対象リポジトリ:

```text
https://github.com/sasakishion0620/sawaPC_shio_MotionController.git
```

## 1. Windows 側: 変更を GitHub に送る

プロジェクトフォルダでターミナルを開いて、次を実行します。

```powershell
git status
git add .
git commit -m "update local changes"
git push
```

### 各コマンドの意味

`git status`
- 今の Git の状態を確認する
- 変更ファイルがあるかどうかを見る

`git add .`
- 今の変更をコミット対象に入れる
- 新しく作ったファイルも含めて登録する

`git commit -m "update local changes"`
- `add` した内容を 1 つの履歴として保存する
- `"update local changes"` は変更内容のメモ

`git push`
- ローカルの履歴を GitHub に送る

### 補足

`git commit` のときに `nothing to commit` と出た場合は、
新しく保存する変更がないという意味です。

## 2. Linux 側: 空のフォルダにコピーする

前提:

- Linux に Git が入っている
- 空のフォルダがすでにある
- その空フォルダの中に、このリポジトリを入れたい

空フォルダでターミナルを開いて、次を実行します。

```bash
git clone https://github.com/sasakishion0620/sawaPC_shio_MotionController.git .
git status
git remote -v
```

### 各コマンドの意味

`git clone https://github.com/sasakishion0620/sawaPC_shio_MotionController.git .`
- GitHub からプロジェクトをダウンロードする
- 最後の `.` は「今いるフォルダに入れる」という意味

`git status`
- 正しく取得できたか確認する
- どのブランチにいるか確認する

`git remote -v`
- このフォルダがどの GitHub リポジトリにつながっているか確認する

## 3. Linux 側: 取得後の確認

次のようになっていれば OK です。

- `git status` で `On branch main` のように出る
- `git remote -v` で `origin` が GitHub の URL になっている
- `src`, `inc`, `config`, `docs` などが見える

ファイル一覧を見たいときは:

```bash
ls
```

## 4. Linux 側: すでにコピー済みのとき

すでに Linux 側にこのフォルダがあるなら、最新を取るだけでよいです。

```bash
git pull
```

これは GitHub にある最新の内容を取ってくるコマンドです。

## 5. VS Code でやる場合

### 空フォルダがすでにある場合

1. VS Code でその空フォルダを開く
2. ターミナルを開く
3. 次を実行する

```bash
git clone https://github.com/sasakishion0620/sawaPC_shio_MotionController.git .
```

ターミナルの開き方:

- `Terminal` -> `New Terminal`
- `Ctrl + Shift + @`
- `Ctrl + Shift + P` -> `Terminal: Create New Terminal`

### 空フォルダをまだ作っていない場合

VS Code の UI からできます。

1. `Ctrl + Shift + P`
2. `Git: Clone` を選ぶ
3. 次の URL を貼る

```text
https://github.com/sasakishion0620/sawaPC_shio_MotionController.git
```

4. 保存先の親フォルダを選ぶ
5. 開く

## 6. よく使う確認コマンド

Git が入っているか確認:

```bash
git --version
```

空フォルダか確認:

```bash
ls -a
```

`.` と `..` 以外にファイルがあると、
`git clone ... .` は失敗することがあります。

## 7. 最短版

### Windows 側

```powershell
git add .
git commit -m "update local changes"
git push
```

### Linux 側

```bash
git clone https://github.com/sasakishion0620/sawaPC_shio_MotionController.git .
git status
git remote -v
```
