# m5stack_example

m5stack_exampleは[ROSではじめるホビーロボット番外編](https://koso2-dan.booth.pm/items/2026421)で紹介する[ros2arduino](https://github.com/ROBOTIS-GIT/ros2arduino)とM5StackでROS2の学習を行うためのサンプルコードです。

## 動作環境
ROS2 Dashing Diademata  
Ubuntu 18.04  
Micro-XRCE-DDS-Agent v1.1.0

## 依存関係
このサンプルコードは次のパッケージに依存しています。合わせてインストールして下さい。
- Micro-XRCE-DDS-Agent（v1.1.0）  
https://github.com/eProsima/Micro-XRCE-DDS-Agent
- nomumu/m5stack_msgs  
https://github.com/nomumu/m5stack_msgs

## インストール方法
ROS2のワークスペースにcloneしてbuildすることで使用できます。

```
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
~/ros2_ws/src$ git clone https://github.com/nomumu/m5stack_example.git
~/ros2_ws/src$ cd ..
~/ros2_ws$ colcon build
　〜ビルド成功した後〜
~/ros2_ws$ source install/setup.bash
```

Micro-XRCE-DDS-Agentは次のようにビルドしインストールします。

```
$ mkdir ~/micro_dds
$ cd ~/micro_dds
~/micro_dds$ git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent
~/micro_dds$ cd Micro-XRCE-DDS-Agent
~/micro_dds/Micro-XRCE-DDS-Agent$ git checkout v1.1.0
~/micro_dds/Micro-XRCE-DDS-Agent$ mkdir build
~/micro_dds/Micro-XRCE-DDS-Agent$ cd build
~/micro_dds/Micro-XRCE-DDS-Agent/build$ cmake ..
~/micro_dds/Micro-XRCE-DDS-Agent/build$ make
~/micro_dds/Micro-XRCE-DDS-Agent/build$ sudo make install
```

## 作成される実行ファイルと実行方法
- plot_example  
Micro-XRCE-DDS経由でM5Stackにデータをプロットするサンプルです。  
`ros2 launch m5stack_example bringup_plot_example_launch.py`  
- stamp_example  
Micro-XRCE-DDS経由でM5Stackにスタンプ画像を分割転送するサンプルです。  
`ros2 launch m5stack_example bringup_stamp_example_launch.py`  

