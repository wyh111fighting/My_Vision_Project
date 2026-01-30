# My Vision Project

## 项目简介
本仓库用于提交视觉方向任务验收材料，包含代码、配置、文档与运行说明。

## 目录结构
```
my_vision_project/
	README.md                # 项目简介、环境、运行指南
	docs/
		development_log.md     # 开发日志（每日记录）
	src/
		my_controller/         # 控制逻辑/算法模块
		scripts/               # 数据采集或自动化脚本
	config/                  # 配置文件（YAML/参数等）
	.gitignore               # 忽略 build、cache 等
```

## 环境要求
- Python 3.8+
- OpenCV
- NumPy

## 快速开始
***第一阶段：生存环境搭建***

**1.1.1系统基座**

1. Ubuntu虚拟机搭建完成

![QQ_1769240939512](assets/QQ_1769240939512-20260124154903-xiil840.png)

2. Xshell

为提升效率，开启Ubuntu系统内远程连接服务SSH，使用主机电脑通过网络连接到虚拟机

并且使用Xshell，连接Ubuntu的SSH系统，后续在Xshell中敲Linux命令用于操作Ubuntu系统，用于提升效率

ssh 192.168.229.128  (连接虚拟机）

![QQ_1769242503398](assets/QQ_1769242503398-20260124161509-03lbrff.png)

3. Xftp

查看文件或者进行文件的传输，较为便利

下载源文件，可先保存至主机，后通过Xftp转入虚拟机内

![QQ_1769251143654](assets/QQ_1769251143654-20260124183907-rwbvs6l.png)

![QQ_1769251297363](assets/QQ_1769251297363-20260124184141-46z8tft.png)

安装ros-noetic-desktop-full时候默认官方源存在情况下，安装失败，删除官方源，全部配置为ustc后正常

4. 控制海龟生成和运动代码：

rosrun turtlesim turtlesim_node

rosrun turtlesim turtle_teleop_key

roscore

5. 安装VScode中问题：

Command 'curl' not found, but can be installed with:

sudo apt install curl

安装后报错，暂时不能解析域名“mirrors.ustc.edu.cn”
E: 无法下http://mirrors.ustc.edu.cn/ubuntu/pool/main/c/curl/curl_7.68.0-1ubuntu2.25_amd64.deb  暂时不能解析域名“mirrors.ustc.edu.cn”

系统无法解析域名mirrors.ustc.edu.cn ，这说明虚拟机存在**网络 DNS 解析故障**，导致连不上 Ubuntu 软件源，既装不了 curl，也没法后续下载微软的 GPG 密钥。

网络故障排查：尝试ping一些常见域名，如www.qq.com发现无法连接，故确认为是虚拟机网络连接出错。将网络模式由NAT模式改为桥接模式后，发现可以正常上网，排查后确定为NAT过程有故障，vmware有部分功能未启动，可能是待机过长导致，启动后恢复正常

找到的安装包是针对Ubuntu18.04版本，20.04版本需要进行改动为neotic后可以正常

**1.1.2机器人仿真部署**

1. 选择 Unitree Go1

2. 缺失部分：

 https://github.com/unitreerobotics/unitree_ros/tree/master

https://github.com/unitreerobotics/unitree\_gazebo

https://github.com/unitreerobotics/unitree\_ros\_to\_real

https://github.com/unitreerobotics/unitree\_ros

https://github.com/unitreerobotics/unitree\_legged\_sdk


重建指令：（便于失败后重新开始）

`mkdir -p /data/catkin_ws/src`

`cd  /data/catkin_ws/src`

`git clone https://github.com/unitreerobotics/unitree_ros `

`git clone https://github.com/unitreerobotics/unitree_ros_to_real `

`git clone https://github.com/unitreerobotics/unitree_legged_sdk`

`git clone https://github.com/unitreerobotics/unitree_guide `

一些概念理解：

ROS（机器人操作系统）—— 机器人的 “大脑操作系统”

Gazebo —— 机器人的 “虚拟测试场”

Unitree Go1 仿真环境 —— 专属 Go1 的 “游戏模组”

![QQ_1769415231112](assets/QQ_1769415231112-20260126161357-rz75sdc.png)

一开始环境配置为stair发现机器人不稳定自己运动，趋向地面，脱离阶梯 改为earth后正常（这个尚未思考原因，可能是环境配置的时候，重力相关参数出错?)

3. gazebo机器狗启动

启动设置：`roslaunch unitree_gazebo normal.launch rname:=go1 wname:=earth`

站立命令：`rosrun unitree_controller unitree_servo`

环境变量：`source ./devel/setup.sh`

工作目录：`ro/data/catkin_ws/`

施加外力：

`rosrun unitree_controller unitree_external_force`

控制站立：

`rosrun unitree_controller unitree_move_kinetic`

4. unitree\_guide

控制器：https://github.com/unitreerobotics/unitree_guide

启动unitree_guide: `./devel/lib/unitree_guide/junior_ctrl`

用法：启动控制器后，机器人会平躺在模拟器的地面上。此时，按下键盘上的“2”键，将机器人的有限状态机（FSM）从**被动**（初始状态）切换到**固定站立状态**；再按下“4”键，将FSM从**固定站立状态**切换到**小跑状态**。现在，您可以按下“w”、“a”、“s”、“d”键控制机器人的平移，按下“j”、“l”键控制机器人的旋转。按下空格键，机器人将停止并站立在地面上。（如果没有反应，您需要点击打开的启动控制器的终端，然后重复上述操作。）

操作细节：

w/s = 前进/后退（X轴速度）
a/d = 左/右平移（Y轴速度）
i/j = 左/右转向（Z轴角速度）

q/e = 身体左/右倾斜
z/c = 身体升高/降低
空格 = 急停（立即停止所有运动）

操作时序：

按 w → 保持2-3秒 → 同时按 w+i → 保持1-2秒 → 松开 i → 保持 w 或松开

修改文档内容方法gedit ctrl S ctrl Q

***第二阶段：ROS 通信与运动控制***

抛弃键盘，用代码接管机器人的神经中枢。

**1.2.1 源码审计与接口改造 (Hard Core)** 

这一步开始聚焦C++源码修改，但是因为我只学习了python，对C++了解不足，引入AI完成，开始使用千问等AI完成，效果不佳，subscriber 订阅部分代码修改或者站立映射部分有误，导致后续1.2.2无法进展，代码无法控制机器狗电机运动，所以全部重新搭建了新的catkin_make重头来过

想到初始下载的VScode,选择使用vscode copilot,发现使用虚拟机打开并不好用，于是建立了虚拟机和主机VScode联系，使主机上VScode连接虚拟机进行操作


catkin\_ws/
├── src/          ← 你放代码的地方
├── build/
├── devel/        ← 编译后生成的环境脚本在这里
└── CMakeLists.tx

**1.2.2 自动化巡逻脚本**

nano编辑

chmod +x

change mode 的缩写，Linux 内置命令，专门用于修改文件 / 目录的访问权限

`+` \= 「添加权限」，`x` \= execute（可执行）的缩写 → 给文件添加可执行权限


Linux 中新建的脚本文件（比如你写的 `patrol.py`），默认只有「读（r）」和「写（w）」权限，**没有可执行（x）权限**。如果直接运行脚本，会提示 `Permission denied`（权限被拒绝）。

***第三阶段：AI 视觉闭环***

**1.3.1 视觉感知系统** 

**1.3.2 视觉伺服 (Visual Servoing)** 

rosrun rqt_image_view rqt_image_view

roslaunch unitree_guide gazeboSim.launch rname:=go1

rosrun unitree_guide visual_servo.py

## 任务验收材料
- `docs/development_log.md`：每天更新调试过程、参数、问题与结论
- 运行说明：在本 README 中补充

## 说明
该结构仅为参考，可根据实际需要增删文件，但务必保持目录整洁清晰。

运行方法见 docs/development_log.md
