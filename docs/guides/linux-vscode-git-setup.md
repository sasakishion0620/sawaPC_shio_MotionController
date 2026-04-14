# Linux + VS Code でこのリポジトリを取得する手順

このメモは、Linux PC 側に空のフォルダがすでにある前提で、
このプロジェクトを GitHub からダウンロードするための手順をまとめたものです。

対象リポジトリ:

```text
https://github.com/sasakishion0620/sawaPC_shio_MotionController.git
```

## 1. 空のフォルダを VS Code で開く

1. VS Code を開く
2. `File` -> `Open Folder...`
3. 用意してある空フォルダを選ぶ

例:

```text
/home/<user>/work/sawaPC_shio_MotionController
```

## 2. VS Code でターミナルを開く

次のどれかでターミナルを開く。

- `Terminal` -> `New Terminal`
- `Ctrl + Shift + @`
- `Ctrl + Shift + P` -> `Terminal: Create New Terminal`

## 3. 空フォルダの中に clone する

VS Code のターミナルで次を実行する。

```bash
git clone https://github.com/sasakishion0620/sawaPC_shio_MotionController.git .
```

最後の `.` は、「今開いている空フォルダの中に展開する」という意味です。

## 4. ちゃんと取れているか確認する

```bash
git status
git remote -v
ls
```

期待する状態:

- `git status` で `On branch main` と出る
- `git remote -v` で `origin` が GitHub の URL になっている
- `ls` で `src`, `inc`, `config` などが見える

## 5. あとで最新を取りたいとき

すでに clone 済みのフォルダでは、以後はこれでよい。

```bash
git pull
```

## 6. VS Code の UI でやる場合

### 初回

空フォルダがあるなら、初回だけはターミナルで次を実行するのが一番わかりやすい。

```bash
git clone https://github.com/sasakishion0620/sawaPC_shio_MotionController.git .
```

### 2回目以降

VS Code の Source Control 画面から更新できる。

1. 左の Source Control アイコンを開く
2. `...` メニューを押す
3. `Pull` を選ぶ

またはコマンドパレットで:

```text
Git: Pull
```

## 7. よくある失敗

### フォルダが空でない

次のコマンドで中身を確認する。

```bash
ls -a
```

`.` と `..` 以外のファイルやフォルダがあれば、`git clone ... .` は失敗することがあります。

### Git が入っていない

```bash
git --version
```

これでバージョンが出なければ、先に Git をインストールする必要があります。

## 8. 最短版

空フォルダを VS Code で開いたあと、ターミナルでこれだけ実行すればよい。

```bash
git clone https://github.com/sasakishion0620/sawaPC_shio_MotionController.git .
git status
git remote -v
```
