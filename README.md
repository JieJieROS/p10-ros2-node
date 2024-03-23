## DDT2_HW

#### 相关配置

##### 硬件

| 类别     | 型号             |
| -------- | ---------------- |
| USB2CAN  | MKS CANable V2.0 |
| 关节电机 | DDT P10 48V      |

##### 软件环境

| 主要环境     | 版本  |
| ------------ | ----- |
| Ubuntu       | 20.04 |
| ROS2         | foxy  |
| ros2_control | foxy  |



#### 项目介绍

- 利用ros2_control框架https://github.com/ros-controls/ros2_control/tree/foxy 实现硬件抽象层HAL。
- 利用Linux socketcan实现can信号的收发



#### 安装

##### ros2_control

```
sudo apt-get install ros-foxy-ros2-control
sudo apt-get install ros-foxy-ros2-controller
```

##### 源码

```
cd ~
mkdir ddt2_hw
cd ddt2_hw
git clone git@github.com:L-SY/ddt2_hw.git ~/ddt2_hw/src
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
```

##### 相关依赖

```
sudo apt-get install ros-foxy-yaml-cpp-vendor
sudo apt-get install ros-foxy-plotjuggler*
sudo apt-get install ros-foxy-rqt*
```



#### 源码介绍

- ddt2_assets：存放机器人（电机）相关配置，如控制器配置，urdf，launch
  - config：ros2_control框架下controller的配置
  - description：
    - ros2_control：urdf的<ros2_control>-tag xacro标签
    - urdf：urdf.xacro文件
  - launch：用launch.py来加载相关ros节点
- ddt2_controller：存放自定义的ros2_control的控制器，通过plugins以插件的形式给ros2_control使用
- ddt2_hw：实现底层的can数据的收发
- ddt2_msgs：用于在ddt2_controller中发布电机的相关信息



#### 使用

##### 初始化MKS CANable V2.0的设置

```
ls /dev | grep ttyACM                         查看USB2CAN工具生成的设备,例如ttyACM0
sudo slcand -o -c -s8 /dev/ttyACMx can0       注意波特率要1M适配电机（-s8),换ttyACM中x为本机
sudo ifconfig can0 up
```

##### 开启控制器

> 目前仓库中为位置控制模式
>
> 如果需要更改控制模式为速度控制则需要更改~/ddt2_ws/src/ddt2_asset/one_actuator/config/controller.yaml中
>
> `command_mode: position`为`velocity`

```c
source ~ddt2_ws/install/local_setup.bash
ros2 launch ddt2_assets load_hw.launch.py
```

如果正常则会生成以下topic

```
/actuator_states
/ddt2_controller/commands
/dynamic_joint_states
/gains/joint1/pid_state
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

##### 控制电机

往`/ddt2_controller/commands`上发命令则可以控制，例如：

```
ros2 topic pub /ddt2_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.5]}"
```

##### 电机信息可视化 

> 利用plotjuggler来看ros topic信息

```
ros2 run plotjuggler plotjuggler
```

`/joint_states`：电机信息的effort,position,velocity

`/actuator_states`：电机的一般信息加电机can信号中的原始信息

![ddt2_hw_plotjuggler.png](docs%2Fddt2_hw_plotjuggler.png)

##### 电机控制器参数修改（PID）

通过rqt来修改pid的参数

```
rqt
```

在左上角Plugins/configuration/DynamicReconfiguration中

![ddt2_hw_rqt.png](docs%2Fddt2_hw_rqt.png)
