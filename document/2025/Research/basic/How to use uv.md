# uv（Python パッケージ管理）のインストールと使い方ガイド

対象: macOS / Linux（bash想定）。Windows の場合も考え方は同じですが、アクティベート手順が異なります。

---

## 1. uv のインストール

### 1.1 公式インストーラ（推奨）

```bash
$ curl -LsSf https://astral.sh/uv/install.sh | sh
```

インストール確認:

```bash
$ uv --version
```

### 1.2 Homebrew（macOS）

```bash
$ brew install uv
$ uv --version
```

---

## 2. uv の基本的な考え方

uv は、プロジェクト単位で

- 依存関係（パッケージ）を **pyproject.toml** に記録
- 依存関係をロック（lock）して再現性を上げる
- `.venv/` の仮想環境を自動で用意し、`uv run` でそこを使って実行

という運用ができます。

---

## 3. 新規プロジェクトでの標準フロー（推奨）

### 3.1 プロジェクト作成・初期化（pyproject + .venv）

```bash
$ mkdir myproj
$ cd myproj
$ uv init
```

### 3.2 パッケージ追加（依存関係に登録）

```bash
$ uv add numpy
$ uv add pandas
```

### 3.3 仮想環境上で実行（アクティベート不要）

```bash
$ uv run python -c "import numpy as np; print(np.__version__)"
```

---

## 4. 仮想環境（.venv）のアクティベート方法

> uv の推奨は **アクティベートせず `uv run` を使う** ことです。  
> ただし、従来どおり手動でアクティベートしたい場合は以下です。

### 4.1 macOS / Linux（bash, zsh）

プロジェクト直下に `.venv/` が存在する前提です。

```bash
$ source .venv/bin/activate
```

終了（元に戻す）:

```bash
$ deactivate
```

### 4.2 Windows（PowerShell）

```powershell
> .venv\Scripts\Activate.ps1
```

---

## 5. 「activate: No such file or directory」と出るとき

エラー例:

```bash
$ source .venv/bin/activate
bash: .venv/bin/activate: No such file or directory
```

原因はほぼ次のどちらかです。

- 今いるディレクトリに `.venv/` がまだ作られていない
- `.venv/` を作った別ディレクトリにいる

確認:

```bash
$ pwd
$ ls -a
```

`.venv/` が無い場合は、プロジェクト直下で以下のいずれかを実行します。

**A. uv 管理プロジェクトとして作る（推奨）**

```bash
$ uv init
```

**B. 従来どおり venv を自分で作る**

```bash
$ python -m venv .venv
```

---

## 6. パッケージのインストール方法（2パターン）

### 6.1 uv 管理プロジェクト（pyproject に記録）: `uv add`

プロジェクトの依存関係として追加し、ロック・同期も行います。

```bash
$ uv add requests
$ uv add "numpy>=2.0"
$ uv add --dev pytest
```

削除:

```bash
$ uv remove requests
```

実行:

```bash
$ uv run pytest
```

---

### 6.2 pip 互換の入れ方（既存 venv で使いたい）: `uv pip`

既にある仮想環境に対して **pip の代わり** として使う想定です。

```bash
$ uv pip install requests
$ uv pip install -r requirements.txt
$ uv pip uninstall requests
```

---

## 7. OpenCV（cv2）の入れ方（よくある注意）

`cv2` は **import 名** であり、PyPI のパッケージ名は通常 `opencv-python` です。  
そのため `uv add cv2` は失敗します。

正しい例:

```bash
$ uv add opencv-python
```

GUI 不要の環境（サーバなど）の例:

```bash
$ uv add opencv-python-headless
```

確認:

```bash
$ uv run python -c "import cv2; print(cv2.__version__)"
```

---

## 8. 最小チートシート

```bash
# インストール
$ curl -LsSf https://astral.sh/uv/install.sh | sh
$ uv --version

# 新規プロジェクト
$ mkdir myproj && cd myproj
$ uv init
$ uv add requests
$ uv run python -c "import requests; print(requests.__version__)"

# 手動アクティベート（必要なら）
$ source .venv/bin/activate
$ python -c "import requests; print(requests.__version__)"
$ deactivate
```
