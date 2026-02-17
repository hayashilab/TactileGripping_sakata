# README

This project is based on [Grasp EveryThing (GET)](https://github.com/GelSight-lab/GraspEveryThing) — a novel 1-DoF, 3-finger gripper design for securely grasping objects. Our work builds upon the GET gripper's hardware and software design.

## Raspberry Pi セットアップ (Imaging Driver)

GETグリッパーの触覚センシング用Raspberry Piのセットアップ手順。

### 1. OSの書き込み (Raspberry Pi Imager)

[Raspberry Pi Imager](https://www.raspberrypi.com/software/)で最新の64bit Raspberry Pi OSを書き込む。
OSカスタマイズ画面で以下を設定：
- ホスト名: `imaging.local`
- ユーザー名・パスワード
- WiFi設定
- ロケール設定（設定しないとWiFiが無効化される）
- **Services**タブで`Enable SSH` + `Use password authentication`を有効化

書き込み後、PCとEthernetで接続し、PC側のIPv4を**Link-Local Only**に設定。
```bash
ssh pi@imaging.local
```

### 2. Imaging Driverのインストール

PC側からPiにコードをコピー：
```bash
bash bash_scripts/sync_code_to_pi.sh
```

Pi上でインストール：
```bash
ssh pi@imaging.local
cd imaging_driver
sudo bash install.sh
```

### 3. ネットワーク設定

Pi上でネットワーク設定：
```bash
sudo nmtui
```
- `Edit a connection` → `Wired Connection`を編集
  - `IPv4 Configuration`: **Link-Local**
  - `IPv6 Configuration`: **Disabled**

### 4. カメラモジュール v3 (imx708) の設定

カメラ認識の確認：
```bash
libcamera-hello --list-cameras
```

認識されない場合、`/boot/firmware/config.txt`を編集：
- `camera_auto_detect=0` を `1` に変更
- `[all]`セクションに追加：
  - Pi 5: `dtoverlay=imx708,cam1`
  - Pi 4以前: `dtoverlay=imx708`

### 5. WDR (Wide Dynamic Range) の有効化（任意）

```bash
# デバイス確認
v4l2-ctl -d /dev/v4l-subdev0 -l

# WDR ON
v4l2-ctl --set-ctrl wide_dynamic_range=1 -d /dev/v4l-subdev5

# WDR OFF
v4l2-ctl --set-ctrl wide_dynamic_range=0 -d /dev/v4l-subdev5
```

### 6. Imaging Driver同期用コマンド

メインPCからRPiのimaging_driverを同期：
```bash
rsync -avz --progress -e 'ssh -p 65535' hayashi-rpi@imaging.local:/home/hayashi-rpi/imaging_driver/ /home/hayashi/worksp/camera_ws/src/grasp_everything/grasp_everything/imaging_driver
```

---

## カメラストリーミング自動起動 (systemd)

RPi起動時にカメラストリーミング(`setup_cam_and_led.sh`)を自動実行するための設定。

### インストール
```bash
sudo cp /home/hayashi-rpi/imaging_driver/run_imaging.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable run_imaging.service
sudo systemctl start run_imaging.service
```

### serviceファイル
`/home/hayashi-rpi/imaging_driver/run_imaging.service`
```ini
[Unit]
Description=Imaging service for LBM tactile sensing
After=multi-user.target network.target

[Service]
Type=simple
WorkingDirectory=/home/hayashi-rpi/imaging_driver
ExecStart=/home/hayashi-rpi/imaging_driver/bash_scripts/setup_cam_and_led.sh
ExecStop=/bin/kill -TERM $MAINPID
Restart=on-failure
RestartSec=5
KillMode=mixed
KillSignal=SIGTERM
TimeoutStopSec=10
StandardOutput=journal
StandardError=journal
User=hayashi-rpi

[Install]
WantedBy=multi-user.target
```

### よく使うコマンド
```bash
# ステータス確認
sudo systemctl status run_imaging.service

# ログ確認（リアルタイム）
sudo journalctl -u run_imaging.service -f

# サービス停止 / 再起動
sudo systemctl stop run_imaging.service
sudo systemctl restart run_imaging.service

# 自動起動無効化
sudo systemctl disable run_imaging.service
```

---

## USB Serial デバイス登録 (udev)

グリッパーモータ用FT232Rのデバイスファイル名を固定する設定。

### 1. デバイス情報の確認
```bash
udevadm info -q property -n /dev/ttyUSB0 | grep -E "ID_SERIAL_SHORT=|ID_VENDOR_ID=|ID_MODEL_ID="
```

| ID        | Value    |
|-----------|----------|
| idVendor  | 0403     |
| idProduct | 6001     |
| serial    | BG00XX91 |

### 2. udevルールファイルの作成
USBを抜いた状態で作成：
```bash
sudo nano /etc/udev/rules.d/99-ft232r-serial.rules
```

内容：
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="BG00XX91", SYMLINK+="FT232R_gripper", MODE="0666"
```

### 3. リロード＆確認
```bash
sudo udevadm control --reload
sudo udevadm trigger
```

USBを接続して確認：
```bash
ls -l /dev/FT232R_gripper
```

参考: [USB-シリアル変換器のデバイスファイル名を固定する](https://smdn.jp/electronics/udev_create_persistent_usb_device_symlink/)

---

その他の情報はdocument/2025/Researchにまとめています。
Cadファイルなどは[Onshape-MyGripper](https://cad.onshape.com/documents/9a01cb18827afc23df61d956/w/22e798bb93dee9ab0cdcc348/e/33dd1a345062008ad6b48713?renderMode=0&uiState=6994188b9e3005ec0ca43f3c)
