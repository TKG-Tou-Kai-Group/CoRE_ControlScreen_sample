# CoRE_ControlScreen_sample

本リポジトリは、[The The Championship of Robotics Engineers](https://core.scramble-robot.org/)（通称CoRE(コア)）の遠隔操縦ロボットに向けた操作画面を生成するシステムをまとめたものである。
180度視野の魚眼カメラ映像を合成して真上から俯瞰して見ているようなアラウンドビュー画像を生成、表示することで操作性の向上を図っている。
また、透過PNG画像を合成できるため、照準の自作やチームのエンブレムの追加が容易にできるようにしている。

# 部品表

| 部品名 | 単価 | 個数 | 合計 | 備考 | 購入リンク |
|-------|------|-----|------|------|------------|
| Raspberry Pi 4B/8GB | 13,970円 | 1台 | 13,970円 | 画像を合成するコンピュータ | [Link](https://raspberry-pi.ksyic.com/?pdp.id=552) |
| ELP-USBFHD04H-L180 | 6,799円 | 2台 | 13,598円 | アラウンドビューのための魚眼カメラ | [Link](https://amzn.asia/d/d2Er8BX) |
| ELP-USB8MP02G-L75-JP | 6,888円 | 1台 | 6,888円 | 正面を見るための標準カメラ | [Link](https://amzn.asia/d/hFUPfFL) |
| マイクロHDMI-HDMIケーブル | 749円 | 1本 | 749円 | HDMIトランスミッタとRaspberry Piを接続するケーブル | [Link](https://amzn.asia/d/7HqSHYs) |
合計：35,205円

ロボットに組み込むにあたって、USB電源を別途用意する必要がある。
利用できるモバイルバッテリは複数あるので、必要な稼働時間やロボットのスペースに応じて選定すること。

プログラムのインストールや変更のために、別途HDMIディスプレイやUSBマウス、キーボードが必要となる。

# カメラ配置
追記予定

# Raspberry Pi のセットアップ方法
## 1. SDカード

1. Raspberry Pi ImagerでRaspberry Pi OS Lite (64-bit,**bullseye**)をMicro SDに書き込む。Imagerの設定でユーザ名**pi**とパスワードを予め設定すること。
2. Micro SDに作成された"bootfs"ドライブ直下に`ssh`という名前のファイルを配置してSSHを有効化する。
3. Raspberry PiにセットアップしたSDカードを挿入し、電源を入れる。

## 2. ROS2のインストール

Raspbian OS用の、[非公式のdebパッケージ](https://github.com/Ar-Ray-code/rpi-bullseye-ros2)を使ってインストールする

```bash
wget https://github.com/Ar-Ray-code/rpi-bullseye-ros2/releases/download/ros2-0.3.1/ros-humble-desktop-0.3.1_20221218_arm64.deb
sudo apt install ./ros-humble-desktop-0.3.1_20221218_arm64.deb
sudo pip install vcstool colcon-common-extensions

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.1. ROS2パッケージのビルド

必要なパッケージのインストール

```bash
sudo apt install libboost-python-dev libopencv-dev python3-opencv
pip3 install numba numpy==1.26.4
```

ソフトウェアのダウンロード

```bash
cd ~
git clone  --recursive https://github.com/TKG-Tou-Kai-Group/CoRE_ControlScreen_sample.git
```

USBカメラのセットアップ

```bash
sudo cp ~/CoRE_ControlScreen_sample/config/99-usb-camera.rules  /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
```

ROS2パッケージのビルド

```bash
cd ~/CoRE_ControlScreen_sample/ros2_ws
colcon build --symlink-install
source install/local_setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 2.2. ROS2ノードの実行
```bash
ros2 launch core_robot_launcher robot_launcher.launch.py
```
- 画面にカメラ映像が出力されれば成功！
- この段階で下記のようなエラーが出る場合は，Raspberry piのOSのバージョンが誤っているためSDカード作成からやり直す必要がある．OSのバージョンはbullseyeを指定すること（何も指定しないと最新版のbookwormがインストールされる）
```[INFO] [launch]: All log files can be found below /home/pi/.ros/log/2024-02-11-18-34-23-302048-raspberrypi-2602
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught exception when trying to load file of format [py]: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/humble/lib/python3.9/site-packages/_rclpy_pybind11.cpython-311-aarch64-linux-gnu.so' isn't present on the system. Please refer to 'https://docs.ros.org/en/humble/Guides/Installation-Troubleshooting.html#import-failing-without-library-present-on-the-system' for possible solutions
```

### 2.3. 自動起動
```bash
sudo ln -s /home/pi/CoRE_ControlScreen_sample/service/scramble_robot_ros2.service /etc/systemd/system
chmod 755 /home/pi/CoRE_ControlScreen_sample/service/ros2_launch.sh
sudo systemctl enable scramble_robot_ros2.service
sudo systemctl start scramble_robot_ros2.service
systemctl status scramble_robot_ros2.service
```

### 2.4. ロボットに合わせた調整
追記予定

## 3. （オプション）SDカードのRead only化
SDカードへの書き込み中に電源を抜くとSDカードが壊れる可能性があるので、read-onlyに設定する。

ただし、ソフトウェアを編集できなくなるので大会直前などでソフトウェアを変更しないときに設定するとよい。

参考: [Raspberry Pi で Overlay File System (read-only file system) を試す](https://qiita.com/nanbuwks/items/d5d0cfc5f94177515a6a)

1. ターミナルで`sudo raspi-config`を実行
2. "4 Performance Options Configure performance settings"
3. "P3 Overlay File System Enable/disable read-only file system"
4. 有効化して再起動


## 4. その他
- ros2_launch.shにて`ROS_DOMAIN_ID=0`に設定しています。
- デフォルトの設定ではIPアドレスの割当はDHCPで行われます。

# Third-party license
本手順で作成した場合、以下のサードパーティ製ソフトウェアがインストールされます。
(本レポジトリに直接は含まれていません)

- Raspbian OS
- ROS2 Humble
- [rpi-bullseye-ros2](https://github.com/Ar-Ray-code/rpi-bullseye-ros2/blob/main/LICENSE) (MIT License)
