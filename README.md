![Workflow Status](https://github.com/SSL-Roots/consai2r2/workflows/ROS2-Dashing/badge.svg)

# CON-SAI2 R2

CON-SAI2 R2 (consai2r2)は[RoboCup SSL](https://ssl.robocup.org/)に 初めて参加する人でも開発できるサッカーAIです。

ROSベースの[consai2](https://github.com/SSL-Roots/consai2)をROS 2へ移行したものです。

**開発中です。移行は完了していません。**

**CON**tribution to **S**occer **AI** by **R**OS **2**

# Requirements

## Hardware

- [ROS2](https://index.ros.org/doc/ros2/)が動くマシン
  - パソコン(PC)と呼ばれるものがおすすめです。
  - 参考スペック
    - Intel(R) Core(TM) i5-6600K CPU @ 3.50GHz
    - 8 GB of RAM
    - 有線LANポート（試合会場では有線LANでロボット・ボール位置座標データを受信します)
- SSLロボット
  - 実世界のロボットを動かす際に必要です
  - [参考：Rootsのロボット](https://github.com/SSL-Roots/Roots_home/wiki/robot_Ver_JapanOpen2019)
- PC - ロボット間の無線通信機器
  - 実世界のロボットを動かす際に必要です
  - RootsはUSB接続の機器を使用しています

## Software
- Ubuntu 18.04
  - Win, OS Xについては動作保証していません
- [ROS 2 Dashing Diademata](https://index.ros.org/doc/ros2/Installation/Dashing/)
  - Desktop Install 推奨です
- [grSim](https://github.com/RoboCup-SSL/grSim)
  - RoboCup SSLの運営が用意したシミュレータソフトです
  - 必須ではありませんがデバッグに役立ちます
- [SSL-Game-Controller](https://github.com/RoboCup-SSL/ssl-game-controller)
  - RoboCup SSLの運営が用意した審判ソフトです
  - 必須ではありませんがデバッグに役立ちます 

# Installation

## ROSのインストール

ROS (Robot Operating System) は、ロボットソフトウェア開発をサポートする ライブラリ・ツールが豊富に含まれたオープンソースソフトウェアです。
ROS 2はROSの新しいバージョンです。

consai2r2はROS 2 Dashingに対応しています。

[Installing ROS2 via Debian Packages](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)
を参考に、インストールしてください。Desktop Install 推奨です。

## consai2r2のクローン&ビルド

```zsh
# ワークスペースの作成
mkdir -p ~/ros2_ws/src

# consai2r2のクローン
cd ~/ros2_ws/src
git clone https://github.com/SSL-Roots/consai2r2

# 依存関係のインストール
# 事前にrosdep install, update を実行すること
rosdep install -r -y --from-paths . --ignore-src

# consai2r2のビルド
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

## consai2r2のデバッグに役立つツールをインストール

grSim(シミュレータアプリケーション)とSSL-Game-Controller(審判アプリケーション)は
RoboCup SSLのソフトウェア開発に役立ちます。

下記ページを参考にアプリケーションをインストールしてください。

- [grSim](https://github.com/RoboCup-SSL/grSim)
- [SSL-Game-Controller](https://github.com/RoboCup-SSL/ssl-game-controller)

# How to Use

## Joystick control

下記コマンドでgrSimのロボットをジョイスティックコントローラで操作できます。

```zsh
ros2 launch consai2r2_examples joystick_example.launch.py
```

# Development

GitHubのProjects機能で開発方針に合わせたタスク管理をしています。

- [1st step - consai2 to consai2r2 porting](https://github.com/SSL-Roots/consai2r2/projects/1)
- [2nd step - SSL Game Example](https://github.com/SSL-Roots/consai2r2/projects/3)
- [3rd step - use ROS2 effectively](https://github.com/SSL-Roots/consai2r2/projects/2)

# Contribution

issue, pull request 大歓迎です。

[CONTRIBUTING.md](./CONTRIBUTING.md)を見てくれると嬉しいです。

# Author & Contributors

consai2r2はRoboCup SSLに参加している日本人チーム**Roots**が立ち上げました。

Roots以外にもconsai2r2の開発に貢献しているメンバーがいます！[Contributors](https://github.com/SSL-Roots/consai2r2/graphs/contributors)のページを見てください。

RoboCup SSLへの参加方法、ロボットに必要な機能、開発環境などは Rootsのホームページに記載してます。

[Roots - Home](https://github.com/SSL-Roots/Roots_home/wiki)

# LICENSE

[MIT LICENSE](./LICENSE)
