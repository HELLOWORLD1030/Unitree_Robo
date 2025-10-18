---
title: 宇树A1折腾日记
urlname: ecx0n7
date: '2021-12-21 23:13:19 +0800'
categories: 折腾记录
sticky: 1
tags: []
abbrlink: 18a05227
---

近日，有幸参加了一个四足机器狗越障挑战赛，体验了一把国内能和波士顿对抗的很酷的公司-宇树科技做的宇树 A1，机器狗的强大的性能让我和队友惊叹，但是折腾 A1 机器狗的过程却是比较曲折，这会是本文的主题

<!-- more -->

# 基本情况介绍

1. 宇树 A1 拥有两块主控板，靠近摄像头的一块板是树莓派，用于实现计算机视觉等功能。靠近机器狗屁股的是 x86 架构的主控板，负责运动控制
2. 宇树 A1 搭载了一台 D435i 的 Intel Realsense 原深感深度相机，可实现目标识别，颜色识别，深度感知等（需配套 opencv 的相关代码）

# 遥控器遥控操作

1. L2+B 激活、休眠电机
2. L2+A 改变机器狗身体高度、锁死机身
3. start 解锁、踏步
4. L1+start 进入运动模式
5. L1+L2+start 退出运动模式
6. L1+A 打招呼（运动模式下）
7. L1+B 滑步（运动模式下）
8. L1+X 跳舞（运动模式下）
9. L1+Y 后空翻（运动模式下）
10. L2+Y 侧翻（运动模式下）
11. L2+X 翻到后重新恢复正确姿态

# 程序控制操作

## 环境安装和配置

前文提到，机器狗搭载了两块主控板，也提供了四个 USB3 的接口，两个 RJ-45 网口，两个 HDMI 视频输出接口，可以提供二次开发的基础支持

### 机器狗与电脑的连接

1. 第一次启动机器狗，需要一台显示器，一套键鼠，将机器狗连接上述外设后（显示屏通过 HDMI 连接到树莓派），开机，机器狗会自检站立，遥控让其趴下，休眠电机

2. 获取树莓派的 ip，也可利用树莓派的无线网卡连接场地的 WiFi 等，总之需要控制机器狗的电脑和机器狗在同一个网段，获取到树莓派的 IP 后，可将电脑连至机器狗的 WIFI，SSID 名为 Unitree-xxxx（编号），密码为 8 个 0，连入之后，电脑和机器狗处在同一个网段，此时可以通过 ssh 连入树莓派

```shell
ssh pi@树莓派ip
password：123
```

也同样可以 ssh 连入机器狗的运动控制主控板

```shell
ssh unitree@192.168.123.161 #IP写死的
password：123
```

### 树莓派依赖安装

```shell
sudo apt-get update && sudo apt-get -y upgrade

sudo apt-get install -y --no-install-recommends python3-setuptools python3-pip python3-dev

sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

> 1. 这一步需要联网，可以将树莓派的 WiFi 连接至可上网的路由器
> 2. 联网后，发现执行 apt-get 还是报错，执行 route 命令后发现，机器狗默认配置一个 default 网关会导致数据包出不去，ping 百度发现，数据在本机环回，因此切换到 root 用户（密码 123）执行 route del default 后切换回 pi 用户，ping 百度，问题解决
> 3. apt-get update 不成功可 apt update 试试 可能有些步骤需要手动确认
> 4. 若出现以下情况

```shell
Reading package lists... Done
W: GPG error: https://packagecloud.io/headmelted/codebuilds/raspbian/: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY 9165938D90FDDD2E
E: The repository 'https://packagecloud.io/headmelted/codebuilds/raspbian/' is not signed.
N: Updating from such a repository can't be done securely, and is therefore disabled by default.
N: See apt-secure(8) manpage for repository creation and user configuration details.
```

> cd 到 /etc/apt/sources.list.d/ 删去 跟此链接有雷同单词的文件[https://packagecloud.io/headmelted/codebuilds/raspbian/](https://packagecloud.io/headmelted/codebuilds/raspbian/) ，重新执行 apt-get update

### 树莓派安装 realsense

```shell
git clone https://github.com/IntelRealSense/librealsense.git
cd ./librealsense
sudo ./scripts/setup_udev_rules.sh
mkdir build && cd build
# Need Internet
 cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3

# Takes long time
make -j2

sudo make install

sudo gedit ~/.bashrc

# Insert the following line at the bottom of the file, then save and exit
export PYTHONPATH=$PYTHONPATH: /usr/lib/python3/dist-packages/pyrealsense2

# Source
source ~/.bashrc
```

> 安装后执行 realsense-viewer，若显示找不到设备，则 cmake 采用
> cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DPYTHON_EXECUTABLE=/usr/bin/python3 -DFORCE_RSUSB_BACKEND=ON
> 重新执行上述步骤

### 树莓派安装 opencv

```shell
pip3 install --upgrade opencv-python
```

> 若一直卡住 Building wheel for opencv-python 则
> pip3 install opencv-python==（低版本的 opencv，我这里采用 4.026 安装成功）

### x86 架构的 upboard 安装 msgpack

```shell
sudo apt-get install libmsgpack*
```

至此，所有依赖安装成功

## demo 脚本测试

> demo 代码 gitee 地址：[https://gitee.com/helloworld0923/Unitree_RoBoCom](https://gitee.com/helloworld0923/Unitree_RoBoCom)
> 本代码从[https://github.com/DonovanZhu/Unitree_RoBoCom](https://github.com/DonovanZhu/Unitree_RoBoCom) 导入而来

![image.png](https://yuque.zzgpro.top/yuque/0/2021/png/1573794/1640274145311-448f9884-62b3-426b-b40e-26ee023876d9.png#averageHue=%23fefefe&clientId=ub609cd8a-d119-4&from=paste&height=83&id=u8e044872&name=image.png&originHeight=165&originWidth=1751&originalType=binary∶=1&rotation=0&showTitle=false&size=17406&status=done&style=none&taskId=u4d6c66ff-888d-4960-b133-7aedc50e6b2&title=&width=875.5)

### Color_Detection.py

用于实现机器狗识别颜色等，运行在树莓派上

> 若出现 No Device Connect，尝试 reboot 树莓派，若不行的话，尝试下上面环境安装关于 realsense 的方案，重新编译

### Realsense_Get_Picture.py

用于调用机器狗上的相机拍照

### Get_HSV.py

对上一步获取到的图片进行 hsv 取样，用于 Color_Dection.py 的颜色标定

### color_detection.cpp

运动控制程序

## 运动控制程序编译

> [https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3](https://github.com/unitreerobotics/unitree_legged_sdk/releases/tag/v3.3)
> github 仓库放在这里

### Dependencies

- [Boost](http://www.boost.org) (version 1.5.4 or higher)
- [CMake](http://www.cmake.org) (version 2.8.3 or higher)
- [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)

```bash
cd lcm-x.x.x
mkdir build
cd build
cmake ../
make
sudo make install
```

### Build

将 color_detection.cpp 放入 examples 文件夹，修改 CMakeList.txt 文件如下

```powershell
add_executable(color_detection examples/color_detection.cpp)
target_link_libraries(color_detection ${EXTRA_LIBS})
```

```bash
mkdir build
cd build
cmake ../
make -j4
```

### run

进入 build 文件夹，运行对应文件，程控机器狗。需要在运动模式下进行，** 且机器狗运动模式程序版本高于 1.19**

## 联合调试

### 电脑连接机器狗 wifi

### windows 上 ssh 远程进入树莓派

### windows 或树莓派 ssh 进入 Upboard

### 树莓派 cd 到对应目录，运行 Color_Dection.py 程序

### UpBoard cd 到对应目录，运行 color_detction

> 至此，树莓派控制 realsense 摄像头识别颜色，并通过 udp 传回控制指令至 upboard，upboard 控制机器狗做出动作，整套流程已完成

## 远程桌面连接树莓派

在颜色标定过程中，有时我们需要观察机器狗传回的画面，此时 SSH 远程就不够用了，我们需要远程桌面连接，这里采用的是微软的 rdp 协议，windows 通过 mstsc 程序连接树莓派

树莓派执行下列命令，如安装遇阻，查看上方树莓派依赖安装章节

```shell
sudo apt-get install xrdp
```

安装好后，windows 运行 mstsc，输入树莓派 ip 即可连接

> 我们手中的机器狗出现了，一旦远程连接，realsense 摄像头便无法正常调用，表现为 No Device Connected，解决方案是 reboot 树莓派，先 ssh 连入树莓派，执行 py 脚本调用摄像头，再远程桌面连接
