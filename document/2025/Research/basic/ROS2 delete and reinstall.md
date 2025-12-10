# ROS2 完全アンインストール手順

## 1. インストール済み ROS / ROS2 の確認

- [ ]  `/opt/ros` に入り、インストール済みディストリビューションを確認する
```
$ cd /opt/ros ls
```

---

## 2. ROS2 パッケージの削除

### ● 全 ROS / ROS2 パッケージ削除

- [x] 全 ROS / ROS2 パッケージを削除
```
$ sudo apt remove ros-*
```

### ● 特定ディストリビューションのみ削除（例：humble）

-  例：humble のみ削除
    

`$ sudo apt remove ros-humble-*`

### ● 不要パッケージの自動削除

- [x] autoremove を実行
`$ sudo apt autoremove`

### ● /opt/ros の残骸削除

- [x] 削除対象ディストリビューションのディレクトリを削除

`$ cd /opt sudo rm -r ros/<distribution_name>`

---

## 3. `.bashrc` のクリーンアップ

-  `.bashrc` を開く
    
-  ROS / ROS2 関連の `source` 行を削除  
    （例：`/opt/ros/.../setup.bash`、`~/ros2_ws/install/setup.bash`）
    

`$ nano ~/.bashrc`

---

## 4. ROS2 ワークスペースの削除（任意）

- [x] 必要なら事前にバックアップ
    
- [x] 不要なら削除

```
$ rm -r ~/ros2_ws
```

---

## 5. VS Code の ROS 拡張を削除（任意）

- [ ] VS Code 内で ROS 関連拡張をアンインストール
    

---

## 6. ROS2 apt ソースの削除

- [x]  ROS2 の apt ソース（ros2.list）を削除
```
$ cd /etc/apt/sources.list.d/ 
$ sudo rm ros2.list
```

- [x] apt を更新
    

```
$ sudo apt update
```

---

## 7. 最終掃除と再起動

- [x]  autoremove を再度実行

```
$ sudo apt autoremove
```

- [ ] PC を再起動
    

```
$ sudo reboot
```

---

# ✔ ROS2 の完全アンインストール完了！

---