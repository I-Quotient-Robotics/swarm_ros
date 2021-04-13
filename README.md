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
    sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
    ```

3. 执行`catkin_make`完成编译

4. 启动机器人的定位导航程序

5. 执行如下指令，启动群控程序

    ```bash
    roslaunch swarm_center bringup.launch
    ```

## 从机程序使用说明

1. 拷贝swarm_ros到从机器人的ROS工作目录内

2. 执行如下指令安装依赖

    ```bash
    sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
    sudo apt install python-pip
    sudo -H pip2 install roslibpy
    ```
    
3. 执行`catkin_make`完成编译

4. 参照[群控程序设置](README.md#群控程序设置)，设置主机器人IP，以及从机队形位置

5. 启动机器人的定位导航程序

6. 执行如下指令，启动群控子程序

    ```bash
    roslaunch swarm_client bringup.launch
    ```

## swarm_center 使用方式

1. swarm_center运行后，会生成一个ros service，名称`swarm_set_command`

2. 可以通过service，设置编队形式，例如：

    ```bash
    rosservice call /swarm_set_command "command: 'follow'"
    ```
3. 默认的swarm_center/bringup.launch，会启动joystick节点，此时可以通过手柄按键，发送设定好的ros service 信息，具体请参考[joy_config.yaml](swarm_center/config/joy_teleop.yaml)

    ```yaml      
    teleop:
      follow:
        type: service
        service_name: /swarm_set_command
        service_request:
          command: 'follow'
        buttons: [0]

      dismiss:
        type: service
        service_name: /swarm_set_command
        service_request:
          command: 'dismiss'
        buttons: [2]

      cover:
        type: service
        service_name: /swarm_set_command
        service_request:
          command: 'cover'
        buttons: [1]
    ```

4. swarm_center还会发送机器人的定位到主题`swarm_base_pose`中

## swarm_client 使用方式

1. 程序启动后，会自动连接到主机器人的ROS上，并订阅`swarm_base_pose`收取主机器人的坐标

2. 获得主机器人坐标后，会在本机的TF中，添加主机器人的位置，以及不同阵型下对应的机器人位置，具体可以通过Rviz中的TF来进行观测

3. swarm_client同时还会将自身的位置，发布到主机器人的ROS中，主题名`swarm_hearbeat`

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
