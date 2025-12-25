# TactileGripping リポジトリ Git 運用メモ

## 1. ローカルリポジトリの作成と初期コミット

### やったこと

- [x] 作業用ディレクトリに移動  
  ```bash
  $ cd ~/worksp
  $ pwd
  $ ls
  ```

- [x] Git リポジトリを初期化  
  ```bash
  $ cd ~/worksp
  $ git init
  ```

- [x] Git のユーザー情報を設定（初回のみ）  
  ```bash
  $ git config user.name "GitHub上の自分の名前"
  $ git config user.email "GitHubに登録しているメールアドレス"
  ```

- [x] `.gitignore` を作成して、ビルド生成物などを除外  
  例（ROS2 ワークスペースを想定）：
  ```text
  # colcon / ROS2 workspace
  build/
  install/
  log/

  # Python
  __pycache__/
  *.pyc
  *.pyo

  # VSCode
  .vscode/
  
  # obsidian
  document/.obsidian
  ```

- [x] すべてのファイルをステージングして初回コミット  
  ```bash
  $ git status
  $ git add .
  $ git commit -m "Initial commit"
  ```

---

## 2. GitHub リポジトリとの接続

### やったこと

- [x] GitHub 上に `YoshitakaSAKATA/TactileGripping` を作成  
  （ブラウザから GitHub で作成）

- [x] リモート `origin` を登録（※ SSH の例）  
  ```bash
  $ cd ~/worksp/TactileGripping
  $ git remote add origin git@github.com:YoshitakaSAKATA/TactileGripping.git
  ```

  HTTPS で行う場合は：
  ```bash
  $ git remote add origin https://github.com/YoshitakaSAKATA/TactileGripping.git
  ```

- [x] ブランチ名を `main` に統一  
  ```bash
  $ git branch -M main
  ```

---

## 3. GitHub への認証設定

### パスワードエラー発生

- [x] `git push` 時にパスワードエラーが発生  
  ```bash
  $ git push -u origin main
  Username for 'https://github.com': UserNAME
  Password for 'https://username@github.com': 
  remote: Invalid username or token. Password authentication is not supported for Git operations.
  fatal: Authentication failed for 'https://github.com/username/repo.git/'
  ```

### 対応方針（どちらかで運用）

- [ ] HTTPS + Personal Access Token（PAT）で push する  
  1. GitHub の Settings → Developer settings → Personal access tokens からトークン発行  
  2. `git push` 時の「Password」にトークン文字列を入力

  ```bash
  $ git push -u origin main
  # Username: UserNAME
  # Password: GitHubで発行したトークン
  ```

- [ ] SSH キーで認証する（今後楽に使えるので推奨）

  SSH キー生成：
  ```bash
  $ ssh-keygen -t ed25519 -C "GitHubのメールアドレス"
  ```

  公開鍵の確認：
  ```bash
  $ cat ~/.ssh/id_ed25519.pub
  ```

  GitHub の「SSH and GPG keys」に公開鍵を登録したあと、接続確認：
  ```bash
  $ ssh -T git@github.com
  ```

  リモート URL を SSH 版に変更：
  ```bash
  $ cd ~/worksp
  $ git remote set-url origin git@github.com:UserNAME/repo.git
  $ git remote -v
  ```

---

## 4. 「fetch first」エラー（リモート側に既に履歴がある場合）

### 発生したエラー

- [x] 初回 `git push` 時に、リモートの `main` に既にコミットがあり拒否された  
  ```bash
  $ git push -u origin main
  To github.com:UserNAME/repo.git
   ! [rejected]        main -> main (fetch first)
  error: failed to push some refs to 
  'github.com:UserNAME/repo.git'
  hint: Updates were rejected because the remote contains work that you do
  hint: not have locally. This is usually caused by another repository pushing
  hint: to the same ref. You may want to first integrate the remote changes
  hint: (e.g., 'git pull ...') before pushing again.
  ```

### 対応パターン

#### パターンA：リモートの変更も取り込んで統合（推奨）

- [ ] リモートの `main` をマージしてから push  
  ```bash
  $ cd ~/worksp
  $ git pull origin main --allow-unrelated-histories
  ```

- [ ] コンフリクトが出た場合はファイルを編集して解消し、再度コミット  
  ```bash
  $ git status
  # コンフリクトしたファイルを修正した後
  $ git add <修正したファイル>
  $ git commit -m "Merge remote main into local main"
  $ git push -u origin main
  ```

#### パターンB：ローカルの履歴で上書き（GitHub側は捨てる）

- [x] リモートに大事な内容がない場合のみ、強制 push  
  ```bash
  $ cd ~/worksp/TactileGripping
  $ git push -u origin main --force
  ```

---

## 5. `.gitignore` と `.obsidian` ディレクトリの扱い

### 問題になった点

- [x] `.gitignore` に `~/document/.obsidian/` を書いた  
  → しかし `.gitignore` は「リポジトリ内の相対パス」で書く必要があり、`~` は展開されない  
- [x] すでに `.obsidian` がリポジトリにコミット済みの場合、  
  → `.gitignore` を書くだけではリモートの `.obsidian` は削除されない

### 正しい `.gitignore` の書き方例

- リポジトリ直下に `.obsidian` がある場合：
  ```text
  .obsidian/
  ```

- `document/.obsidian/` という構造の場合：
  ```text
  document/.obsidian/
  ```

### 既に追跡されている `.obsidian` を「Git管理からだけ」外す手順

- [x] `.gitignore` に正しいパスを追加したうえで、Git の追跡から外す  
  （ローカルのファイルは削除しない）

  例：リポジトリ直下の `.obsidian` を外す場合：
  ```bash
  $ cd /path/to/your/repo
  $ git rm -r --cached .obsidian
  $ git status
  $ git commit -m "Stop tracking .obsidian directory"
  $ git push origin main
  ```

  例：`document/.obsidian` の場合：
  ```bash
  $ cd /path/to/your/repo
  $ git rm -r --cached document/.obsidian
  $ git status
  $ git commit -m "Stop tracking document/.obsidian directory"
  $ git push origin main
  ```

---

## 6. 今後の基本的な Git 運用フロー

### 普段の作業サイクル

- [ ] 作業前に最新化  
  ```bash
  $ cd ~/worksp/TactileGripping
  $ git pull origin main
  ```

- [ ] コード修正 / ファイル追加 / 設定変更などを行う

- [ ] 動作確認（例：ROS2 のビルドなど）  
  ```bash
  $ colcon build
  ```

- [ ] 変更内容の確認  
  ```bash
  $ git status
  ```

- [ ] 変更をステージング  
  ```bash
  $ git add <変更したファイル>
  # まとめてでよければ
  $ git add .
  ```

- [ ] 意味のある単位でコミット  
  ```bash
  $ git commit -m "ここに分かりやすいメッセージ"
  ```

- [ ] GitHub へ push  
  ```bash
  $ git push origin main
  ```

---

## 7. 参考：ブランチを切って作業する流れ（大きな変更用）

- [ ] `main` を最新にしてから新しいブランチを作成  
  ```bash
  $ cd ~/worksp
  $ git switch main
  $ git pull origin main
  $ git switch -c feature/xxxxx
  ```

- [ ] ブランチ上で作業 → コミット → ブランチを push  
  ```bash
  $ git add .
  $ git commit -m "Implement xxxxx"
  $ git push -u origin feature/xxxxx
  ```

- [ ] 機能が安定したら `main` にマージして push  
  ```bash
  $ git switch main
  $ git pull origin main
  $ git merge feature/xxxxx
  $ git push origin main
  ```

---
