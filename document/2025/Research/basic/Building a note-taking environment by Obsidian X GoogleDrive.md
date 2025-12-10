

作業日：2025-12-05  
環境：Ubuntu + Obsidian

---

## ✅ 作業チェックリスト

- [x] OneDrive クライアントで初回認証を実施  
  (- <span style="color:#2196f3">install時にTerminalにでた出力結果です</span> )  
  - ブラウザで Microsoft アカウントにログインし、リダイレクト URI をターミナルに貼り付けて認証。

- [x] `onedrive --synchronize` 実行時のログ内容を確認  
  (- <span style="color:#2196f3">onedrive --synchronize のログについて説明してください</span> )  
  - `curl 7.81.0` の HTTP/2 バグ警告を確認（HTTP/1.1 強制により動作自体は問題なし）。
  - OneDrive API からのアイテム取得中に `Ctrl + C` で中断していたことを確認。

- [x] OneDrive のフォルダツリーを確認する方法を整理  
  (- <span style="color:#2196f3">OneDrive のツリーを調べる方法が知りたい</span> )  
  - `onedrive --display-remote-tree` でリモートツリーを一覧表示できることを確認。  
  - `onedrive --sync --dry-run --verbose` で同期対象パスの確認ができることも整理。

- [x] OneDrive クライアントの削除方法を確認  
  (- <span style="color:#2196f3">onedriveをubuntuから削除する方法</span> )  
  - `$ sudo apt purge onedrive`  
  - `$ rm -r ~/.config/onedrive` で設定削除  
  - `$ systemctl --user stop/disable onedrive` で自動起動停止

- [x] Obsidian と OneDrive の相性問題を確認  
  (- <span style="color:#2196f3">現状onedriveをsystemctlを使って動悸させています。Obsidianでメモを作ったとき動機が失敗して safeBackup ファイルが作られるのを避けたい</span> )  
  - Obsidian の頻繁な書き込みにより、  
    `Research-hayashi-Dell-G15-Special-Edition-5521-safeBackup-0001`  
    のような競合ファイルが発生するリスクを認識。
  - 対策として「クラウド直下に Vault を置かない方針」を検討。

- [x] Google Drive などの選択肢を比較し、rclone 利用方針を決定  
  (- <span style="color:#2196f3">GoogleDriveを使ったほうがいいですか？</span> )  
  - Ubuntu では公式 Google Drive クライアントがないため、  
    rclone + ローカル Vault + 一方向バックアップ構成を採用する方針に。

- [x] rclone のインストール（公式スクリプト）  
  (- <span style="color:#2196f3">rcloneでやりたい</span> / <span style="color:#2196f3">https://rclone.org/downloads/ このサイトを参考にしてください</span> )  
  - `curl https://rclone.org/install.sh | sudo bash`  
  - `rclone version` でバージョン確認。

- [x] rclone で Google Drive remote `gdrive` を作成  
  (- <span style="color:#2196f3">Option service_account_file. この項目はなに？</span> / <span style="color:#2196f3">advanced configはどうしたらいい？</span> / <span style="color:#2196f3">Use web browser to automatically authenticate rclone with remote? ここは何？</span> / <span style="color:#2196f3">Configure this as a Shared Drive (Team Drive)?</span> / <span style="color:#2196f3">Keep this "gdrive" remote?</span> )  
  - `rclone config` で新規 remote 作成。  
  - `service_account_file` は空のまま（Enter）。  
  - `Use advanced config?` は `n`。  
  - `Use web browser to automatically authenticate?` は `y` を選択し、ブラウザで認証。  
  - `Configure this as a Shared Drive (Team Drive)?` は `n`（個人のマイドライブ用）。  
  - 最後に `Keep this "gdrive" remote?` は `y` で保存。

- [x] Google アカウント切り替え方法を確認  
  (- <span style="color:#2196f3">googleアカウントを変更したい場合</span> )  
  - 新しい remote を作る方法（例：`gdrive2`）と、  
    `rclone config reconnect gdrive:` で再認証する方法を把握。

- [x] rclone でバックアップコマンドを試行し、エラー原因を確認  
  (- <span style="color:#2196f3">rclone copy 実行時のエラー（Command copy needs 2 arguments maximum...）これで作業終了ですか？</span> )  
  - 実行コマンド：  
    `rclone copy ./worksp/document gdrive: obsidian-backup --verbose`  
  - 引数の書き方ミスによりエラー発生（`gdrive:` と `obsidian-backup` の間の空白が原因）。

- [ ] 正しいコマンドでのバックアップ実行（今後の作業）  
  (- <span style="color:#2196f3">これまでの作業をobsidianに記録しておくために質問内容も含めてmdファイルに出力してください</span> / <span style="color:#2196f3">質問した結果出た作業内容にはその後に質問内容を付け加えて</span> )  
  - 修正コマンド：  
    `rclone copy ./worksp/document gdrive:obsidian-backup --verbose`  
  - まずは `--dry-run` で確認：  
```
$ rclone copy ./worksp/document gdrive:obsidian-backup --verbose --dry-run
```

---

## 📝 今後の運用メモ（Obsidian 用）

- Obsidian の Vault は **ローカルディレクトリ**（例：`./worksp/document`）に置く。  
- クラウド側は **rclone で一方向バックアップ**（例：`gdrive:obsidian-backup`）。  
- OneDrive の常時監視同期（`onedrive --monitor` や systemd 自動同期）は  
  Obsidian Vault には使わない（safeBackup などの競合ファイルを避けるため）。  
- バックアップは手動または systemd timer で `rclone copy` を定期実行する。
