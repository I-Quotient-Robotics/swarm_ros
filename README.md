# swarm_ros

ROS package for robot swarm

## 简介

- swarm_center：运行在主机器人上的程序
- swarm_client：运行在从机器人上的程序
- swarm_msgs：定义了一些群控中使用到的消息和服务

## 主机程序使用说明

1. 拷贝swarm_ros到主机器人的ros工作目录内

2. 执行如下指令安装ROS依赖

    ```bash
    sudo apt install ros-kinetic-rosbridge-suite
    ```

3. 执行`catkin_make`完成编译

4. 执行如下指令，启动群控程序

    ```bash
    roslaunch swarm_center bringup.launch
    ```

## 从机程序使用说明

1. 拷贝swarm_ros到从机器人的ROS工作目录内

2. 执行如下指令安装依赖

    ```bash
    sudo apt intstall ros-kinetic-rosbridge-suite
    sudo apt install python-pip
    sudo pip2 install roslibpy
    ```
    
3. 执行`catkin_make`完成编译

4. 参照[群控程序设置](README.md#群控程序设置)，设置主机器人IP，以及从机队形位置

## 群控程序设置

1. 更改`swarm_client.py`中的参数部分

    ```python
    # base robot ip and port
    self.__base_port = 9090
    self.__base_ip = '192.168.0.2'

    # robot name, each robot should have different name
    self.__robot_name = "robot_1"
    self.__current_command = "dismiss"

    # follow position
    self.__follow_translation = (-1.5, 1.0, 0.0)
    self.__follow_rotation = (0.0, 0.0, 0.0, 1.0)

    # cover position
    self.__cover_translation = (1.5, 0.0, 0.0)
    self.__cover_rotation = (0.0, 0.0, 0.0, 1.0)

    # dismiss position
    self.__dismiss_ready = False
    self.__dismiss_translation = (0.0, 1.5, 0.0)
    self.__dismiss_rotation = (0.0, 0.0, 0.0, 1.0)
    ```
