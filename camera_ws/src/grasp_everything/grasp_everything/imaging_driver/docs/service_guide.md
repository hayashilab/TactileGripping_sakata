# Imaging Driver サービス管理ガイド

## 概要

Raspberry Pi起動時にカメラサーバーとLED制御を自動実行するsystemdサービス構成です。

---

## サービス構成

| サービス | 説明 | 実行ファイル |
|---------|------|-------------|
| `run_imaging.service` | カメラサーバー | `/usr/bin/python3 run.py` |
| `ws2812.service` | LED制御 | `venvs/ws2812/bin/python ws2812.py` |

### 依存関係

```
run_imaging.service
    │
    ├── Requires=ws2812.service  → 起動時にLEDも起動
    │
    └── PartOf (ws2812側)        → 停止時にLEDも停止
```

---

## 基本コマンド

### サービスの起動

```bash
sudo systemctl start run_imaging.service
```
→ カメラサーバーとLEDが両方起動します

### サービスの停止

```bash
sudo systemctl stop run_imaging.service
```
→ カメラサーバーとLEDが両方停止します

### サービスの再起動

```bash
sudo systemctl restart run_imaging.service
```

### ステータス確認

```bash
sudo systemctl status run_imaging.service ws2812.service
```

### ログ確認（リアルタイム）

```bash
sudo journalctl -u run_imaging.service -u ws2812.service -f
```

### ログ確認（過去ログ）

```bash
sudo journalctl -u run_imaging.service -u ws2812.service --no-pager
```

---

## 自動起動設定

### 自動起動を有効化

```bash
sudo systemctl enable run_imaging.service
sudo systemctl enable ws2812.service
```

### 自動起動を無効化

```bash
sudo systemctl disable run_imaging.service
sudo systemctl disable ws2812.service
```

---

## LEDの手動制御

### LEDを消灯

```bash
sudo /home/hayashi-rpi/venvs/ws2812/bin/python /home/hayashi-rpi/imaging_driver/ws2812_turn_off.py
```

---

## インストール / アンインストール

### インストール

```bash
cd /home/hayashi-rpi/imaging_driver/systemd
./install.sh
```

### アンインストール

```bash
cd /home/hayashi-rpi/imaging_driver/systemd
./uninstall.sh
```

### 手動インストール

```bash
sudo cp /home/hayashi-rpi/imaging_driver/systemd/*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ws2812.service run_imaging.service
```

---

## トラブルシューティング

### LEDが消えない場合

```bash
# 手動で消灯
sudo /home/hayashi-rpi/venvs/ws2812/bin/python /home/hayashi-rpi/imaging_driver/ws2812_turn_off.py
```

### サービスが起動しない場合

```bash
# エラーログを確認
sudo journalctl -u run_imaging.service -n 50 --no-pager
sudo journalctl -u ws2812.service -n 50 --no-pager
```

### 設定変更後の反映

```bash
sudo systemctl daemon-reload
sudo systemctl restart run_imaging.service
```

---

## ファイル構成

```
imaging_driver/
├── run.py                    # カメラサーバー
├── ws2812.py                 # LED制御
├── ws2812_turn_off.py        # LED消灯スクリプト
└── systemd/
    ├── run_imaging.service   # カメラサービス定義
    ├── ws2812.service        # LEDサービス定義
    ├── install.sh            # インストールスクリプト
    └── uninstall.sh          # アンインストールスクリプト
```
