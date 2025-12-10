# ROS 2 Humble インストール手順チェックリスト

## 1. Locale 設定
- [x] UTF-8 が有効か確認する  
```bash
$ locale
```

- [x] locale パッケージのインストール  
```bash
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8
```

- [x] 設定確認  
```bash
$ locale
```

---

## 2. ROS 2 APT ソース設定

### Universe リポジトリ有効化
- [x] software-properties-common をインストール  
```bash
$ sudo apt install software-properties-common
```

- [x] Universe を追加  
```bash
$ sudo add-apt-repository universe
```

### ROS 2 apt ソース追加
- [x] curl をインストールし、ROS apt source を取得  
```bash
$ sudo apt update && sudo apt install curl -y
$ export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
$ curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
$ sudo dpkg -i /tmp/ros2-apt-source.deb
```

---

## 3. ROS 2 パッケージインストール準備

- [x] apt キャッシュ更新  
```bash
$ sudo apt update
```

- [x] 推奨: システムアップグレード  
```bash
$ sudo apt upgrade
```

---

## 4. ROS 2 インストール

### デスクトップ版（推奨）
- [x] デスクトップ版をインストール  
```bash
$ sudo apt install ros-humble-desktop
```

### 最小構成版（GUIなし）
- [x] ROS-Base をインストール  
```bash
$ sudo apt install ros-humble-ros-base
```

### 開発ツール
- [x] 開発ツールのインストール  
```bash
$ sudo apt install ros-dev-tools
```

---

## 5. 環境設定
- [x] ROS 2 環境を読み込み  
```bash
$ source /opt/ros/humble/setup.bash
```

---

## 6. 動作確認（Talker / Listener）

### ターミナル1
- [x] C++ Talker 起動  
```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_cpp talker
```

### ターミナル2
- [x] Python Listener 起動  
```bash
$ source /opt/ros/humble/setup.bash
$ ros2 run demo_nodes_py listener
```

---

## 7. アンインストール

### ROS 2 Humble パッケージ削除
- [ ] Humble の削除  
```bash
$ sudo apt remove ~nros-humble-* && sudo apt autoremove
```

### ROS 2 apt ソース削除
- [ ] apt ソース削除  
```bash
$ sudo apt remove ros2-apt-source
$ sudo apt update
$ sudo apt autoremove
$ sudo apt upgrade
```