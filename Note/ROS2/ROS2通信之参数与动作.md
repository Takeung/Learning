- [基础篇——控制概述](#基础篇控制概述)
  - [1 开关控制与闭环控制](#1-开关控制与闭环控制)
    - [1.1 开环控制介绍](#11-开环控制介绍)
    - [1.2 闭环控制介绍](#12-闭环控制介绍)
    - [1.3 哪个更常用？](#13-哪个更常用)
- [入门篇——参数与动作](#入门篇参数与动作)
  - [1 ROS2参数通信介绍](#1-ros2参数通信介绍)
    - [1.1 参数通信是什么？](#11-参数通信是什么)
      - [1.1.1 参数定义](#111-参数定义)
      - [1.1.2 参数组成成分](#112-参数组成成分)
    - [1.2 体验参数](#12-体验参数)
      - [1.2.1 运行小乌龟模拟器节点和控制节点](#121-运行小乌龟模拟器节点和控制节点)
      - [1.2.2 查看节点有哪些参数（设置）](#122-查看节点有哪些参数设置)
      - [1.2.3 查看参数值](#123-查看参数值)
      - [1.2.4 设置参数](#124-设置参数)
      - [1.2.5 把参数存起来](#125-把参数存起来)
        - [1.2.5.1 给乌龟模拟器参数拍照](#1251-给乌龟模拟器参数拍照)
        - [1.2.5.2 恢复参数值](#1252-恢复参数值)
        - [1.2.5.3 启动节点时加载参数快照](#1253-启动节点时加载参数快照)
  - [2 参数之RCLCPP实现](#2-参数之rclcpp实现)
    - [2.1 创建功能包和节点](#21-创建功能包和节点)
    - [2.2 RCLCPP参数API](#22-rclcpp参数api)
    - [2.3 使用参数控制节点日志级别](#23-使用参数控制节点日志级别)
    - [2.4 编译测试](#24-编译测试)
  - [3 参数之RCLPY实现](#3-参数之rclpy实现)
    - [3.1 创建功能包和节点](#31-创建功能包和节点)
    - [3.2 RCLPY参数API](#32-rclpy参数api)
    - [3.3 使用参数控制节点日志级别](#33-使用参数控制节点日志级别)
    - [3.4 编译测试](#34-编译测试)
  - [4 动作（Action）通信与自定义接口](#4-动作action通信与自定义接口)
    - [4.1 Action背景](#41-action背景)
    - [4.2 Action的组成部分](#42-action的组成部分)
    - [4.3 感受Action](#43-感受action)
      - [4.3.1 启动乌龟模拟器和键盘控制节点](#431-启动乌龟模拟器和键盘控制节点)
      - [4.3.2 使用绝对旋转(Action)控制小乌龟](#432-使用绝对旋转action控制小乌龟)
    - [4.4 Action的CLI工具](#44-action的cli工具)
      - [4.4.1 action list](#441-action-list)
      - [4.4.2 action info](#442-action-info)
      - [4.4.3 action send\_goal](#443-action-send_goal)
    - [4.5 自定义通信接口](#45-自定义通信接口)
      - [4.5.1 创建接口功能包](#451-创建接口功能包)
      - [4.5.2 编写接口](#452-编写接口)
      - [4.5.3 编译生成接口](#453-编译生成接口)
  - [5 动作之CPP实现](#5-动作之cpp实现)
    - [5.1 创建功能包和节点](#51-创建功能包和节点)
      - [5.1.1 创建功能包](#511-创建功能包)
      - [5.1.2 robot.h](#512-roboth)
      - [5.1.3 robot.cpp](#513-robotcpp)
      - [5.1.4 action\_robot\_01.cpp](#514-action_robot_01cpp)
      - [5.1.5 action\_control\_01.cpp](#515-action_control_01cpp)
      - [5.1.6 CMakeList.txt](#516-cmakelisttxt)
    - [5.2 编写机器人类](#52-编写机器人类)
      - [5.2.1 robot.h](#521-roboth)
      - [5.2.2 robot.cpp](#522-robotcpp)
    - [5.3 编写机器人节点](#53-编写机器人节点)
    - [5.4 编写控制节点](#54-编写控制节点)
    - [5.5 编译测试](#55-编译测试)
  - [6 动作之RCLPY实现](#6-动作之rclpy实现)
    - [6.1 创建功能包和节点](#61-创建功能包和节点)
      - [6.1.1 创建功能包](#611-创建功能包)
      - [6.1.2 robot.py](#612-robotpy)
      - [6.1.3 action\_robot\_02.py](#613-action_robot_02py)
      - [6.1.4 action\_control\_02.py](#614-action_control_02py)
      - [6.1.5 setup.py](#615-setuppy)
    - [6.2 编写机器人类](#62-编写机器人类)
    - [6.3 编写机器人节点](#63-编写机器人节点)
    - [6.4 编写控制节点](#64-编写控制节点)
    - [6.5 构建测试](#65-构建测试)
  - [7 通信机制对比总结](#7-通信机制对比总结)
    - [7.1 话题](#71-话题)
    - [7.2 服务](#72-服务)
    - [7.3 参数](#73-参数)
    - [7.4 动作](#74-动作)
- [进阶篇——原理进阶](#进阶篇原理进阶)
  - [1 ROS参数通信原理介绍](#1-ros参数通信原理介绍)
- [2 生命周期节点](#2-生命周期节点)
  - [2.1 生命周期节点介绍](#21-生命周期节点介绍)


# 基础篇——控制概述

## 1 开关控制与闭环控制

机器人的控制可以分为两种类型：开环控制和闭环控制。

### 1.1 开环控制介绍

开环控制是指机器人按照预先设定的命令执行任务，但并不会对执行过程中的状态进行反馈和调整。简单来说，开环控制就是机器人盲目地按照指令执行任务，不考虑实际执行情况是否符合预期。开环控制的优点是**简单易用，适用于一些简单的任务**，如基本的运动控制或简单的搬运。但是，它的缺点也很明显，因为机器人无法感知执行任务的实际情况，因此无法自动调整行动，导致执行任务的**成功率低，可靠性差**。

### 1.2 闭环控制介绍

闭环控制是指机器人通过传感器或其他检测设备获取执行任务过程中的状态信息，将这些信息反馈给控制系统，从而实现对机器人执行任务过程中的实时控制和调整。在闭环控制中，机器人执行任务的过程中会**根据反馈信息调整执行动作**，确保机器人按照预期的方式完成任务。闭环控制的优点是能够根据实际情况进行实时调整，提高了机器人执行任务的成功率和可靠性。

### 1.3 哪个更常用？

在机器人应用中，闭环控制更加普遍，因为它能够根据反馈信息实时调整机器人的动作，确保机器人按照预期的方式执行任务。同时，闭环控制还可以帮助机器人适应不同的工作环境，增强机器人的鲁棒性和自适应能力，实现更高效、更精准的控制。在一些高精度和复杂的应用场景中，闭环控制已经成为机器人控制的标准，比如精密加工、医疗手术、自动驾驶等。



# 入门篇——参数与动作

## 1 ROS2参数通信介绍

在前面的机器人控制示例中我们，机器人每移动一步休息500ms，假如我们想让机器人休息时间少一些，就需要手动地修改源码减少其值，这非常的不方便。

在机器人开发中，会有很多参数和设置可以后期需要调整的，如果都放到源码里很难实现动态修改和管理，ROS2为了解决这一问题，提出了参数这一通信机制。

### 1.1 参数通信是什么？

#### 1.1.1 参数定义

ROS2官方对参数的定义是：

**A parameter is a configuration value of a node. You can think of parameters as node settings.**

**参数是节点的一个配置值，你可以认为参数是一个节点的设置**

#### 1.1.2 参数组成成分

ROS2参数是由键值对组成的，键值对指的是就是名字和数值，比方说

- 名字：李四写小说周期，值：5s
- 名字：显示器亮度,值：60%

名字的数据类型肯定是字符串，值的数据类型呢？我们这里用到的是5，整形数据，显然只有一个整形是不够用的，ROS2支持的参数值的类型如下：

- bool 和bool[]，布尔类型用来表示开关，比如我们可以控制雷达控制节点，开始扫描和停止扫描。
- int64 和int64[]，整形表示一个数字，含义可以自己来定义，这里我们可以用来表示李四节点写小说的周期值
- float64 和float64[]，浮点型，可以表示小数类型的参数值
- string 和string[]，字符串，可以用来表示雷达控制节点中真实雷达的ip地址
- byte[]，字节数组，这个可以用来表示图片，点云数据等信息

### 1.2 体验参数

我们使用乌龟模拟器来体验一下参数，同时讲解一下常用的参数的命令行工具。

#### 1.2.1 运行小乌龟模拟器节点和控制节点

打开终端

```bash
ros2 run turtlesim turtlesim_node
```

再打开一个终端

```bash
ros2 run turtlesim turtle_teleop_key
```

可以看到下面的蓝蓝的模拟器

![image-20210903142558024](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903142558024.png)

#### 1.2.2 查看节点有哪些参数（设置）

我们可以使用下面的指令来查看所有节点的参数列表，打开一个终端，运行下面的指令

```bash
ros2 param list
```

![image-20210903125400440](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903125400440.png)

写代码为什么要做到见名知意？我们看到乌龟模拟器的四个参数，background背景bgr指的是blue、green、red。简而言之就是背景颜色。那这几个参数应该可以控制乌龟模拟器的背景颜色。

> 最后一个use_sim_time是每个节点都带的，含义如下：
>
> 在 ROS 2 中，参数 `use_sim_time` 是一个全局参数，用于控制节点是否使用仿真时间。它主要用于仿真环境下，比如 Gazebo 等仿真工具。
>
> ### `use_sim_time` 参数的作用
>
> - **默认情况下**，`use_sim_time` 设置为 `false`，这意味着节点将使用系统时间（即计算机的实际时间）来标记消息的时间戳。
> - **当设置为 `true` 时**，节点会使用仿真时间（通常由仿真器发布到 `/clock` 主题）来标记消息的时间戳。
>
> ### 使用场景
>
> - **真实机器人**：`use_sim_time` 通常设置为 `false`，因为你希望节点使用系统时间来进行消息时间戳的记录。
> - **仿真环境**：`use_sim_time` 设置为 `true`，这样节点可以与仿真器保持同步，使用仿真时间来进行消息的时间戳记录。这在仿真测试和开发过程中非常重要，因为仿真器可以加速、减速甚至暂停时间。
>
> ### 设置 `use_sim_time`
>
> 可以通过以下几种方式设置 `use_sim_time` 参数：
>
> #### 1. 在启动文件中设置
>
> 在 ROS 2 的 launch 文件中，可以通过参数配置来设置 `use_sim_time`：
>
> ```python
> from launch import LaunchDescription
> from launch_ros.actions import Node
> 
> def generate_launch_description():
>     return LaunchDescription([
>         Node(
>             package='your_package_name',
>             executable='your_executable_name',
>             name='your_node_name',
>             output='screen',
>             parameters=[{'use_sim_time': True}]
>         )
>     ])
> ```
>
> #### 2. 通过命令行设置
>
> 你也可以在命令行启动节点时设置 `use_sim_time`：
>
> ```sh
> ros2 run your_package_name your_executable_name --ros-args -p use_sim_time:=true
> ```
>
> #### 3. 在参数文件中设置
>
> 可以将参数写在 YAML 文件中，并在启动节点时加载这个参数文件：
>
> **params.yaml**：
> ```yaml
> your_node_name:
>   ros__parameters:
>     use_sim_time: true
> ```
>
> 然后在启动文件中加载这个参数文件：
>
> ```python
> from launch import LaunchDescription
> from launch_ros.actions import Node
> 
> def generate_launch_description():
>     return LaunchDescription([
>         Node(
>             package='your_package_name',
>             executable='your_executable_name',
>             name='your_node_name',
>             output='screen',
>             parameters=['params.yaml']
>         )
>     ])
> ```
>
> ### 检查 `use_sim_time` 的设置
>
> 你可以通过以下命令来检查某个节点的 `use_sim_time` 参数设置：
>
> ```sh
> ros2 param get /your_node_name use_sim_time
> ```
>
> ### 典型应用
>
> 1. **导航和定位**：在仿真环境中进行导航和定位测试时，确保所有相关节点使用一致的仿真时间。
> 2. **传感器数据处理**：仿真传感器数据的时间戳与仿真器的时间保持一致，以确保数据的时序正确。
>
> 设置 `use_sim_time` 参数可以帮助你在仿真环境中更准确地模拟真实机器人的行为。

如果看不懂，还可以有一个方法详细查看一个参数的信息。

```bash
ros2 param describe <node_name> <param_name>
```

比如：

```bash
ros2 param describe /turtlesim background_b
```

![image-20210903150815050](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903150815050.png)

这里就可以详细的看到参数的名字，参数的描述，参数的类型，还有对参数的约束，最大值最小值等。

#### 1.2.3 查看参数值

参数的组成由名字和值（键值组成），名字可以通过`param list`获取，值该使用指令获取呢？

下面这个命令行工具可以帮助我们获取参数的值

```bash
ros2 param get /turtlesim background_b
```

运行一下，你会发现结果是255，蓝色进度条打满，再看看r红色和g绿色。

![image-20210903142905553](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903142905553.png)

分别是255，86，69

#### 1.2.4 设置参数

找到了参数和值，接着我们来改变一下乌龟模拟器的颜色。

打开在线工具：[https://fishros.com/tools/pickr](https://fishros.com/tools/pickr/)

选取一个自己喜欢的颜色，比如选绿色，因为乌龟模拟器换成绿色的应该很奇怪。

![image-20210903145055215](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903145055215.png)

可以看到当前的这个颜色：r为44，g为156，b为10，接着我们可以使用下面的指令来设置参数的值。

```bash
ros2 param set <node_name> <parameter_name> <value>
```

我们依次修改参数值：

```bash
ros2 param set /turtlesim background_r 44
ros2 param set /turtlesim background_g 156
ros2 param set /turtlesim background_b 10
```

接着你可以看到这样的颜色的乌龟模拟器（绿的令人发慌）

![image-20210903145702524](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903145702524.png)

> 需要留意的是，我们修改的背景数据并没有被存储，只是临时修改。重新启动节点乌龟模拟器依然还是原来的蓝色，不是我们想要的绿色的。

#### 1.2.5 把参数存起来

把参数存起来其实就相当去把当前的参数值拍一张快照，然后保存下来，后面可以用于恢复参数到当前的数值。

可以使用下面的命令进行操作：

```bash
ros2 param dump <node_name>
```

##### 1.2.5.1 给乌龟模拟器参数拍照

比如我们要保存乌龟模拟器的节点数据，可以采用下面的指令;

```bash
ros2 param dump /turtlesim
```

![image-20210903150318055](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903150318055.png)

文件被保存成了yaml格式，用cat指令看一看

```bash
cat ./turtlesim.yaml
```

![image-20210903150423697](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903150423697.png)

##### 1.2.5.2 恢复参数值

我们`Ctrl+C`关闭乌龟模拟器，然后再重新运行。

```bash
ros2 run turtlesim turtlesim_node
```

![image-20210903151103098](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903151103098.png)

可以看到模拟器又变成了蓝色了，接着通过param的load的方法把参数值恢复成我们之前存储的。

```bash
ros2 param load /turtlesim ./turtlesim.yaml
```

![image-20210903151328843](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903151328843.png)

几乎是瞬间，乌龟模拟器又被我们搞绿了。

##### 1.2.5.3 启动节点时加载参数快照

有什么办法一开始就让乌龟模拟器变成绿色？答案有的。

ros2 的run 指令支持下面这种`骚操作`。

```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

关闭我们的乌龟模拟器，使用下面的指令重新运行

```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
```

![image-20210903151639091](https://fishros.com/d2lros2/humble/chapt4/get_started/1.%E5%8F%82%E6%95%B0%EF%BC%88Param%EF%BC%89%E9%80%9A%E4%BF%A1/imgs/image-20210903151639091.png)

可以看到一上来就时绿了的模拟器。

## 2 参数之RCLCPP实现

上节我们通过参数控制了小乌龟模拟器的背景色，但是我们并不知道小乌龟模拟器是如何接收到参数并将其应用的，本节我们就学习使用ROS2的RCLCPP中参数相关的API实现对ROS2打印的日志级别控制。

ROS2将日志分为五个级别，在RCLCPP中通过不同的宏可以实现不同日志级别日志的打印，例程如下：

```cpp
RCLCPP_DEBUG(this->get_logger(), "我是DEBUG级别的日志，我被打印出来了!");
RCLCPP_INFO(this->get_logger(), "我是INFO级别的日志，我被打印出来了!");
RCLCPP_WARN(this->get_logger(), "我是WARN级别的日志，我被打印出来了!");
RCLCPP_ERROR(this->get_logger(), "我是ERROR级别的日志，我被打印出来了!");
RCLCPP_FATAL(this->get_logger(), "我是FATAL级别的日志，我被打印出来了!");
```

有时候日志太多，会让人眼花缭乱找不到重要信息，所以我们需要对日志的级别进行过滤，比如只看INFO以上级别的，ROS2中可以通过已有的API设置日志的级别，RCLCPP中API如下：

```cpp
this->get_logger().set_level(log_level);
```

### 2.1 创建功能包和节点

我们创建一个功能包和测试节点，声明参数并实现动态修改打印的日志级别功能。

```bash
mkdir -p chapt4/chapt4_ws/
ros2 pkg create example_parameters_rclcpp --build-type ament_cmake --dependencies rclcpp --destination-directory src --node-name parameters_basic
```

**parameters_basic.cpp**

```c++
#include <chrono>
#include "rclcpp/rclcpp.hpp"

class ParametersBasicNode : public rclcpp::Node {
 public:
  explicit ParametersBasicNode(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
  }
 private:
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<ParametersBasicNode>("parameters_basic");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

构建测试

```bash
colcon build --packages-select example_parameters_rclcpp
source install/setup.bash
ros2 run example_parameters_rclcpp parameters_basic
```

### 2.2 [RCLCPP参数API](https://docs.ros2.org/latest/api/rclcpp/)

在RCLCPP的API中，关于参数相关的函数比较多些，但都是围绕[参数获取](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html#a25890d01a2cd47ce99af887f556c529b)、[参数设置](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html#a7d8af4dc449c7130ccc396814b86c14d)、[参数描述](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html#adbfb47c0983e14482c39159b274f6308)、[列出参数](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html#a37aa95886a706c174db77c2b160f6d7d)、[添加](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html#a12d535bced9f26b65c0a450e6f40aff8)和[移除](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html#a15d4f7fa0ef760941b6a78f42cccb7e0)参数回调事件。

### 2.3 使用参数控制节点日志级别

```cpp
#include <chrono>
#include "rclcpp/rclcpp.hpp"
/*
    # declare_parameter            声明和初始化一个参数
    # describe_parameter(name)  通过参数名字获取参数的描述
    # get_parameter                通过参数名字获取一个参数
    # set_parameter                设置参数的值
*/
class ParametersBasicNode : public rclcpp::Node {
 public:
  // 构造函数,有一个参数为节点名称
  explicit ParametersBasicNode(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    this->declare_parameter("rcl_log_level", 0);     /*声明参数*/
    this->get_parameter("rcl_log_level", log_level); /*获取参数*/
    /*设置日志级别*/
    this->get_logger().set_level((rclcpp::Logger::Level)log_level);
    using namespace std::literals::chrono_literals;
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ParametersBasicNode::timer_callback, this));
  }

 private:
  int log_level;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    this->get_parameter("rcl_log_level", log_level); /*获取参数*/
    /*设置日志级别*/
    this->get_logger().set_level((rclcpp::Logger::Level)log_level);
    std::cout<<"======================================================"<<std::endl;
    RCLCPP_DEBUG(this->get_logger(), "我是DEBUG级别的日志，我被打印出来了!");
    RCLCPP_INFO(this->get_logger(), "我是INFO级别的日志，我被打印出来了!");
    RCLCPP_WARN(this->get_logger(), "我是WARN级别的日志，我被打印出来了!");
    RCLCPP_ERROR(this->get_logger(), "我是ERROR级别的日志，我被打印出来了!");
    RCLCPP_FATAL(this->get_logger(), "我是FATAL级别的日志，我被打印出来了!");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<ParametersBasicNode>("parameters_basic");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

**代码解析**

这里我们使用了三个参数相关的函数和一个设置节点日志级别的函数

- declare_parameter，参数有两个参数名和参数值。
- get_parameter，参数有两个，参数名和放入结果的变量。

**设置日志级别**

- set_level，设置日志级别，ROS2的日志级别定义在文件`/opt/ros/humble/include/rcutils/rcutils/logging.h`的167-175行。

  ```cpp
  /// The severity levels of log messages / loggers.
  enum RCUTILS_LOG_SEVERITY
  {
    RCUTILS_LOG_SEVERITY_UNSET = 0,  ///< The unset log level
    RCUTILS_LOG_SEVERITY_DEBUG = 10,  ///< The debug log level
    RCUTILS_LOG_SEVERITY_INFO = 20,  ///< The info log level
    RCUTILS_LOG_SEVERITY_WARN = 30,  ///< The warn log level
    RCUTILS_LOG_SEVERITY_ERROR = 40,  ///< The error log level
    RCUTILS_LOG_SEVERITY_FATAL = 50,  ///< The fatal log level
  };
  ```

### 2.4 编译测试

```bash
colcon build --packages-select example_parameters_rclcpp
source install/setup.bash
ros2 run example_parameters_rclcpp parameters_basic
```

运行后你会发现DEBUG级别的日志并没有被打印出来，原因在于我们将节点的日志级别设置为了0，0对应的日志级别为`RCUTILS_LOG_SEVERITY_UNSET`即未设置使用默认级别，**节点默认的日志级别就是INFO级别**的，所以只能打印INFO以上的日志信息。

![image-20220614013509655](https://fishros.com/d2lros2/humble/chapt4/get_started/2.%E5%8F%82%E6%95%B0%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614013509655.png)

运行节点的时候可以指定参数的值，我们尝试将`log_level`的值改成`10`即`DEBUG`级别。

```shell
ros2 run example_parameters_rclcpp parameters_basic --ros-args -p rcl_log_level:=10
```

![image-20220614020525157](https://fishros.com/d2lros2/humble/chapt4/get_started/2.%E5%8F%82%E6%95%B0%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614020525157.png)

再试试其他级别-FATAL

![image-20220614020607314](https://fishros.com/d2lros2/humble/chapt4/get_started/2.%E5%8F%82%E6%95%B0%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614020607314.png)

除了在节点运行前通过CLI传递参数，在运动的过程中也可以动态的修改参数

```bash
#查看参数列表
ros2 param list 
#设置参数级别
ros2 param set /parameters_basic rcl_log_level 10
```

![image-20220614020835167](https://fishros.com/d2lros2/humble/chapt4/get_started/2.%E5%8F%82%E6%95%B0%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614020835167.png)

> 上面我们通过参数实现了动态控制节点日志级别的功能，其实像这样的功能ROS2早已为我们准备好了，在运行任意节点时候可以通过CLI传递日志级别配置。
>
> ```bash
> ros2 run package-name node-name --ros-args --log-level debug
> ```
>
> 除了命令行设置参数和查看日志，通过rqt也可以可视化设置和查看
>
> ![image-20220614022058258](https://fishros.com/d2lros2/humble/chapt4/get_started/2.%E5%8F%82%E6%95%B0%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614022058258.png)

## 3 参数之RCLPY实现

### 3.1 创建功能包和节点

```bash
cd chapt4/chapt4_ws/
ros2 pkg create example_parameters_rclpy --build-type ament_python --dependencies rclpy --destination-directory src --node-name parameters_basic
```

`parameters_basic.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParametersBasicNode(Node):
    """
    创建一个ParametersBasicNode节点，并在初始化时输出一个话
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ParametersBasicNode("parameters_basic")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

### 3.2 [RCLPY参数API](https://docs.ros2.org/latest/api/rclpy/api/node.html)

![image-20220614103129915](https://fishros.com/d2lros2/humble/chapt4/get_started/3.%E5%8F%82%E6%95%B0%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220614103129915.png)

### 3.3 使用参数控制节点日志级别

```python
class ParametersBasicNode(Node):
    """
    创建一个ParametersBasicNode节点，并在初始化时输出一个话
    """

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")
        # 声明参数
        self.declare_parameter('rcl_log_level', 0)
        # 获取参数
        log_level = self.get_parameter("rcl_log_level").value
        # 设置参数
        self.get_logger().set_level(log_level)
        # 定时修改
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """定时器回调函数"""
        # 获取参数
        log_level = self.get_parameter("rcl_log_level").value
        # 设置参数
        self.get_logger().set_level(log_level)
        print(
            f"========================{log_level}=============================")
        self.get_logger().debug("我是DEBUG级别的日志，我被打印出来了!")
        self.get_logger().info("我是INFO级别的日志，我被打印出来了!")
        self.get_logger().warn("我是WARN级别的日志，我被打印出来了!")
        self.get_logger().error("我是ERROR级别的日志，我被打印出来了!")
        self.get_logger().fatal("我是FATAL级别的日志，我被打印出来了!")
```

### 3.4 编译测试

```bash
colcon build  --packages-select example_parameters_rclpy
source install/setup.bash
ros2 run example_parameters_rclpy parameters_basic
```

![image-20220614103631463](https://fishros.com/d2lros2/humble/chapt4/get_started/3.%E5%8F%82%E6%95%B0%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220614103631463.png)

**指定参数值测试**

```bash
ros2 run example_parameters_rclpy parameters_basic --ros-args -p rcl_log_level:=10
```

**动态设置参数测试**

```bash
ros2 param list
ros2 param set /parameters_basic rcl_log_level 40
```

![image-20220614103936286](https://fishros.com/d2lros2/humble/chapt4/get_started/3.%E5%8F%82%E6%95%B0%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220614103936286.png)

## 4 动作（Action）通信与自定义接口

### 4.1 Action背景

前面章节学习了话题、服务、参数。

**话题**适用于**节点间单向的频繁的数据传输**，**服务**则适用于**节点间双向的数据传递**，而参数则用于**动态调整节点的设置**，动作Action和他们三个有什么不同之处呢？

如果这些问题体现在机器人上，可能是这样子的。我们通过服务服务发送一个目标点给机器人，让机器人移动到该点：

- 你不知道机器人有没有处理移动到目标点的请求（不能确认服务端接收并处理目标）
- 假设机器人收到了请求，你不知道机器人此时的位置和距离目标点的距离（没有反馈）
- 假设机器人移动一半，你想让机器人停下来，也没有办法通知机器人

上面的场景在机器人控制当中经常出现，比如控制导航程序，控制机械臂运动，控制小乌龟旋转等，很显然单个话题和服务不能满足我们的使用，因此ROS2针对控制这一场景，基于原有的话题和服务，设计了动作（Action）这一通信方式来解决这一问题。

### 4.2 Action的组成部分

知道了Action的出现原因，接着说说Action的三大组成部分目标、反馈和结果。

- 目标：即Action客户端告诉服务端要做什么，服务端针对该目标要有响应。解决了不能确认服务端接收并处理目标问题
- 反馈：即Action服务端告诉客户端此时做的进度如何（类似与工作汇报）。解决执行过程中没有反馈问题
- 结果：即Action服务端最终告诉客户端其执行结果，结果最后返回，用于表示任务最终执行情况。

> 参数是由服务构建出来了，而Action是由话题和服务共同构建出来的（一个Action = 三个服务+两个话题） 三个服务分别是：1.目标传递服务 2.结果传递服务 3.取消执行服务 两个话题：1.反馈话题（服务发布，客户端订阅） 2.状态话题（服务端发布，客户端订阅）

![../_images/行动-单一行动.gif](https://fishros.com/d2lros2/humble/chapt4/get_started/4.%E5%8A%A8%E4%BD%9C%EF%BC%88Action%EF%BC%89%E9%80%9A%E4%BF%A1%E4%B8%8E%E8%87%AA%E5%AE%9A%E4%B9%89%E6%8E%A5%E5%8F%A3/imgs/Action-SingleActionClient.gif)

### 4.3 感受Action

带着前面对Action的了解，接着我们一起来了直观的通过小乌龟的案例来感受一下Action的魅力。

#### 4.3.1 启动乌龟模拟器和键盘控制节点

乌龟模拟器

```bash
ros2 run turtlesim turtlesim_node
```

键盘控制节点

```bash
ros2 run turtlesim turtle_teleop_key
```

打开键盘控制节点后，你应该窗口中可以看到下面的提示

```bash
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```

小乌龟键盘控制节点，提供两种可选的控制方式。

- 方向键，通过话题(Topic)控制小乌龟的（直接发送移动话题）
- 绝对旋转，则是采用动作(Action）来控制的小乌龟

#### 4.3.2 使用绝对旋转(Action)控制小乌龟

使用绝对旋转控制小乌龟即使用Action来控制小乌龟。

在小乌龟的遥控窗口我们使用键盘上的F按键周围的按键来尝试运行控制下小乌龟的方向，你会看到小乌龟根据我们所按下按键所在的方向来在原地进行旋转。

同时在旋转的过程中，我们可以使用F按键，来取消小乌龟的运动。

### 4.4 Action的CLI工具

Action的命令行工具一共有三个,下面我们一一介绍。

#### 4.4.1 action list

该命令用于获取目前系统中的action列表。

```bash
ros2 action list
```

你将看到

```bash
/turtle1/rotate_absolute
```

如果在list后加入-t参数，即可看到`action`的类型

```bash
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

知道了接口类型之后，可以使用接口相关CLI指令获取接口的信息

```bash
ros2 interface show turtlesim/action/RotateAbsolute 
```

结果

```bash
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

#### 4.4.2 action info

查看action信息，在终端中输入下面的指令。

```bash
ros2 action info /turtle1/rotate_absolute 
```

你将看到`action`客户端和服务段的数量以及名字。

```bash
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```

#### 4.4.3 action send_goal

该指令用于发送action请求到服务端，我们可以模拟下，让小乌龟转到我们定义的角度。

我们只需要把goal发给服务端即可，根据goal的定义，我们可以看到goal是由一个浮点类型的theta组成的，我们把theta发给服务端。

**发送弧度制1.6给小乌龟**

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.6}"
```

**结果**

```bash
Waiting for an action server to become available...
Sending goal:
     theta: 1.6

Goal accepted with ID: b215ad060899444793229171e76481c7

Result:
    delta: -1.5840003490447998

Goal finished with status: SUCCEEDED
```

我们可以看到goal和result，但是没有看到feedback，这里我们需要加一个参数 --feedback

加上这个参数我们再发送一次。

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.5}" --feedback
```

可以看到，这次的日志中多出了很多实时的反馈，反馈的数值是小乌龟当前的角度与我们给定的目标角度之间的差值，可以看到一直在变小。

```bash
Waiting for an action server to become available...
Sending goal:
     theta: 1.5

Feedback:
    remaining: -0.0840003490447998

Goal accepted with ID: b368de0ed1a54e00890f1b078f4671c8

Feedback:
    remaining: -0.06800031661987305

Feedback:
    remaining: -0.05200028419494629

Feedback:
    remaining: -0.03600025177001953

Feedback:
    remaining: -0.020000219345092773

Feedback:
    remaining: -0.004000186920166016

Result:
    delta: 0.08000016212463379

Goal finished with status: SUCCEEDED
```

### 4.5 自定义通信接口

我们接下来以控制机器人移动到点为例子，来学习Action通信。因为这个场景是我们自己定义的，ROS2并没有拿来就用的接口，所以我们需要自定义Action通信接口。

#### 4.5.1 创建接口功能包

创建功能包

```bash
cd chapt4_ws/
ros2 pkg create robot_control_interfaces --build-type ament_cmake --destination-directory src
```

创建接口文件

```bash
mkdir -p src/robot_control_interfaces/action
touch src/robot_control_interfaces/action/MoveRobot.action
```

packages.xml

```xml
  <depend>rosidl_default_generators</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

CMakeLists.txt

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveRobot.action"
)
```

#### 4.5.2 编写接口

```c++
# Goal: 要移动的距离
float32 distance
---
# Result: 最终的位置
float32 pose
---
# Feedback: 中间反馈的位置和状态
float32 pose
uint32 status
uint32 STATUS_MOVEING = 3
uint32 STATUS_STOP = 4
```

#### 4.5.3 编译生成接口

```bash
colcon build --packages-select robot_control_interfaces
```

编译成功后，即可看到C++头文件和Python相关文件

- C++ Action消息头文件目录`install/robot_control_interfaces/include`
- Python Action消息文件目录`install/robot_control_interfaces/local/lib/python3.10`

## 5 动作之CPP实现

### 5.1 创建功能包和节点

#### 5.1.1 创建功能包

创建example_action_rclcpp功能包，添加`robot_control_interfaces`、`rclcpp_action`、`rclcpp`三个依赖，自动创建`action_robot_01`节点，并手动创建`action_control_01.cpp`节点。

```bash
cd chapt4_ws/
ros2 pkg create example_action_rclcpp --build-type ament_cmake --dependencies rclcpp rclcpp_action robot_control_interfaces --destination-directory src --node-name action_robot_01
touch src/example_action_rclcpp/src/action_control_01.cpp
```

接着我们创建Robot类的头文件和CPP文件。

```bash
touch src/example_action_rclcpp/include/example_action_rclcpp/robot.h
touch src/example_action_rclcpp/src/robot.cpp
```

创建完成后目录结构

```bash
.
├── CMakeLists.txt
├── include
│   └── example_action_rclcpp
│       └── robot.h 
├── package.xml
└── src
    ├── action_control_01.cpp
    ├── action_robot_01.cpp
    └── robot.cpp

3 directories, 6 files
```

#### 5.1.2 robot.h

```cpp
/*
copyright
*/
#ifndef EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
#define EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
#include "rclcpp/rclcpp.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class Robot {
 public:
  Robot() = default;
  ~Robot() = default;
 private:
};

#endif  // EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
```

#### 5.1.3 robot.cpp

暂时为空，第二部分编写

```cpp
#include "example_action_rclcpp/robot.h"
```

#### 5.1.4 action_robot_01.cpp

```cpp
#include "example_action_rclcpp/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

// 创建一个ActionServer类
class ActionRobot01 : public rclcpp::Node {
 public:
  explicit ActionRobot01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<ActionRobot01>("action_robot_01");
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}
```

#### 5.1.5 action_control_01.cpp

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class ActionControl01 : public rclcpp::Node {
 public:
  explicit ActionControl01(std::string name): Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
  }
};  // class ActionControl01

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<ActionControl01>("action_robot_cpp");
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}
```

#### 5.1.6 CMakeList.txt

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_control_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

# action_robot节点
add_executable(action_robot_01 
    src/robot.cpp
    src/action_robot_01.cpp
)
target_include_directories(action_robot_01 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(action_robot_01 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  action_robot_01
  "rclcpp"
  "rclcpp_action"
  "robot_control_interfaces"
  "example_interfaces"
)

install(TARGETS action_robot_01
  DESTINATION lib/${PROJECT_NAME})

# action_control节点
add_executable(action_control_01 
  src/action_control_01.cpp
)
target_include_directories(action_control_01 PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(action_control_01 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  action_control_01
  "rclcpp"
  "rclcpp_action"
  "robot_control_interfaces"
  "example_interfaces"
)

install(TARGETS action_control_01
DESTINATION lib/${PROJECT_NAME})
```

### 5.2 编写机器人类

机器人类主要负责移动机器人和提供当前机器人的状态，我们设计几个函数来实现该功能。

#### 5.2.1 robot.h

```cpp
#ifndef EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
#define EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
#include "rclcpp/rclcpp.hpp"
#include "robot_control_interfaces/action/move_robot.hpp"

class Robot {
 public:
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  Robot() = default;
  ~Robot() = default;
  float move_step(); /*移动一小步，请间隔500ms调用一次*/
  bool set_goal(float distance); /*移动一段距离*/
  float get_current_pose();
  int get_status();
  bool close_goal(); /*是否接近目标*/
  void stop_move();  /*停止移动*/

 private:
  float current_pose_ = 0.0;             /*声明当前位置*/
  float target_pose_ = 0.0;              /*目标距离*/
  float move_distance_ = 0.0;            /*目标距离*/
  std::atomic<bool> cancel_flag_{false}; /*取消标志*/
  int status_ = MoveRobot::Feedback::STATUS_STOP;
};

#endif  // EXAMPLE_ACTIONI_RCLCPP_ROBOT_H_
```

#### 5.2.2 robot.cpp

```cpp
#include "example_action_rclcpp/robot.h"

/*移动一小步，请间隔500ms调用一次*/
float Robot::move_step() {
  int direct = move_distance_ / fabs(move_distance_);
  float step = direct * fabs(target_pose_ - current_pose_) *
               0.1; /* 每一步移动当前到目标距离的1/10*/
  current_pose_ += step;
  std::cout << "移动了：" << step << "当前位置：" << current_pose_ << std::endl;
  return current_pose_;
}

/*移动一段距离*/
bool Robot::set_goal(float distance) {
  move_distance_ = distance;
  target_pose_ += move_distance_;

  /* 当目标距离和当前距离大于0.01同意向目标移动 */
  if (close_goal()) {
    status_ = MoveRobot::Feedback::STATUS_STOP;
    return false;
  }
  status_ = MoveRobot::Feedback::STATUS_MOVEING;
  return true;
}

float Robot::get_current_pose() { return current_pose_; }
int Robot::get_status() { return status_; }
/*是否接近目标*/
bool Robot::close_goal() { return fabs(target_pose_ - current_pose_) < 0.01; }
void Robot::stop_move() {
  status_ = MoveRobot::Feedback::STATUS_STOP;
} /*停止移动*/
```

### 5.3 编写机器人节点

```cpp
class ActionRobot01 : public rclcpp::Node {
 public:
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ServerGoalHandle<MoveRobot>;

  explicit ActionRobot01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    using namespace std::placeholders;  // NOLINT
    this->action_server_ = rclcpp_action::create_server<MoveRobot>(
        this, "move_robot",
        std::bind(&ActionRobot01::handle_goal, this, _1, _2),
        std::bind(&ActionRobot01::handle_cancel, this, _1),
        std::bind(&ActionRobot01::handle_accepted, this, _1));
  }

 private:
  Robot robot;
  rclcpp_action::Server<MoveRobot>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const MoveRobot::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with distance %f",
                goal->distance);
    (void)uuid;
    if (fabs(goal->distance > 100)) {
      RCLCPP_WARN(this->get_logger(), "目标距离太远了，本机器人表示拒绝！");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(),
                "目标距离%f我可以走到，本机器人接受，准备出发！",
                goal->distance);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    robot.stop_move(); /*认可取消执行，让机器人停下来*/
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute_move(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "开始执行移动 %f 。。。", goal->distance);

    auto result = std::make_shared<MoveRobot::Result>();
    rclcpp::Rate rate = rclcpp::Rate(2);
    robot.set_goal(goal->distance);
    while (rclcpp::ok() && !robot.close_goal()) {
      robot.move_step();
      auto feedback = std::make_shared<MoveRobot::Feedback>();
      feedback->pose = robot.get_current_pose();
      feedback->status = robot.get_status();
      goal_handle->publish_feedback(feedback);
      /*检测任务是否被取消*/
      if (goal_handle->is_canceling()) {
        result->pose = robot.get_current_pose();
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Publish Feedback"); /*Publish feedback*/
      rate.sleep();
    }

    result->pose = robot.get_current_pose();
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveRobot> goal_handle) {
    using std::placeholders::_1;
    std::thread{std::bind(&ActionRobot01::execute_move, this, _1), goal_handle}
        .detach();
  }
};
```

**代码解析**

上面的代码信息量有些大，但都是围绕着Action展开的。

首先找到创建Action的API：https://docs.ros2.org/latest/api/rclcpp_action/

![image-20220614121830945](https://fishros.com/d2lros2/humble/chapt4/get_started/5.%E5%8A%A8%E4%BD%9C%E4%B9%8BCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614121830945.png)

Action使用了三个回调函数，分别用于处理收到目标、收到停止、确认接受执行。

- handle_goal，收到目标，反馈是否可以执行该目标，可以则返回`ACCEPT_AND_EXECUTE`,不可以则返回`REJECT`
- handle_cancel，收到取消运行请求，可以则返回`ACCEPT`，不可以返回`REJECT`。
- handle_accepted，处理接受请求，当handle_goal中对移动请求`ACCEPT`后则进入到这里进行执行，这里我们是单独开了个线程进行执行`execute_move`函数，目的是避免阻塞主线程。

执行函数`execute_move`，调用机器人，进行一步步的移动，这里我们采用了while循环的形式，不断调用机器人移动并获取机器人的位置，通过feedback进行反馈。同时检测任务是否被取消，如顺利执行完成则反馈最终结果。

代码中我们还用到了Rate函数来精准控制循环的周期，让其保持为2HZ。

### 5.4 编写控制节点

先看API：[rclcpp_action: rclcpp_action Namespace Reference (ros2.org)](https://docs.ros2.org/latest/api/rclcpp_action/namespacerclcpp__action.html#a464f73efea313d6adb57cbbe5f487f18)

![image-20220614123312541](https://fishros.com/d2lros2/humble/chapt4/get_started/5.%E5%8A%A8%E4%BD%9C%E4%B9%8BCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614123312541.png)

接着看代码

```cpp
class ActionControl01 : public rclcpp::Node {
 public:
  using MoveRobot = robot_control_interfaces::action::MoveRobot;
  using GoalHandleMoveRobot = rclcpp_action::ClientGoalHandle<MoveRobot>;

  explicit ActionControl01(
      std::string name,
      const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
      : Node(name, node_options) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    this->client_ptr_ =
        rclcpp_action::create_client<MoveRobot>(this, "move_robot");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&ActionControl01::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = MoveRobot::Goal();
    goal_msg.distance = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<MoveRobot>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ActionControl01::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&ActionControl01::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&ActionControl01::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  rclcpp_action::Client<MoveRobot>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(GoalHandleMoveRobot::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleMoveRobot::SharedPtr,
      const std::shared_ptr<const MoveRobot::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback current pose:%f", feedback->pose);
  }

  void result_callback(const GoalHandleMoveRobot::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: %f", result.result->pose);
    // rclcpp::shutdown();
  }
};  // class ActionControl01

```

**代码解析**

创建客户端，发送请求的时候可以指定三个回调函数：

- `goal_response_callback`，目标的响应回调函数。
- `feedback_callback`，执行过程中进度反馈接收回调函数。
- `result_callback`，最终结果接收的回调函数。

这里利用了定时器完成了定时请求的功能，请求一次后就立马使用`timer_->cancel();`取消掉了这个定时器，如此就实现了节点启动后定时发一次请求的功能。

### 5.5 编译测试

一个终端，运行机器人节点

```bash
cd chapt4_ws/
colcon build --packages-up-to example_action_rclcpp
source install/setup.bash 
ros2 run example_action_rclcpp  action_robot_01
```

新终端，运行控制节点

```bash
source install/setup.bash 
ros2 run example_action_rclcpp action_control_01
```

![image-20220614124220470](https://fishros.com/d2lros2/humble/chapt4/get_started/5.%E5%8A%A8%E4%BD%9C%E4%B9%8BCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614124220470.png)

执行完成

![image-20220614124248774](https://fishros.com/d2lros2/humble/chapt4/get_started/5.%E5%8A%A8%E4%BD%9C%E4%B9%8BCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220614124248774.png)

> 此前第3章的教程给这里埋了一个[坑](https://fishros.org.cn/forum/topic/2144/%E6%B1%82%E5%8A%A9-%E5%8A%A8%E6%89%8B%E5%AD%A6ros2-action%E5%AE%9E%E7%8E%B0-%E7%AB%A0%E8%8A%82%E4%B8%AD-%E8%BF%90%E8%A1%8C%E6%8E%A7%E5%88%B6%E8%8A%82%E7%82%B9%E6%97%B6%E5%87%BA%E7%8E%B0segmentation-fault%E9%94%99%E8%AF%AF%E6%97%A0%E6%B3%95%E8%A7%A3%E5%86%B3)：ROS2内置的DDS会与手动编译安装的FastDDS存在冲突，导致段错误，例程无法运行。
>
> 解决方法：**把创建的fastdds_ws工作空间删除，然后去~/.bashrc文件中配置的环境变量删除即可**

## 6 动作之RCLPY实现

### 6.1 创建功能包和节点

#### 6.1.1 创建功能包

```bash
cd chapt4_ws/
ros2 pkg create example_action_rclpy --build-type ament_python --dependencies rclpy robot_control_interfaces --destination-directory src --node-name action_robot_02
# 手动再创建action_control_02节点文件
touch src/example_action_rclpy/example_action_rclpy/action_control_02.py
#手动创建机器人类robot.py
touch src/example_action_rclpy/example_action_rclpy/robot.py
```

#### 6.1.2 robot.py

```python
from robot_control_interfaces.action import MoveRobot
import math

class Robot():
    """机器人类，模拟一个机器人"""

    def __init__(self) -> None:
        pass

    def get_status(self):
        """获取状态"""
        pass

    def get_current_pose(self):
        """获取当前位置"""
        pass

    def close_goal(self):
        """接近目标"""
        pass

    def stop_move(self):
        """停止移动"""
        pass

    def move_step(self):
        """移动一小步"""
        pass

    def set_goal(self, distance):
        """设置目标"""
        pass
```

#### 6.1.3 action_robot_02.py

```python
#!/usr/bin/env python3

import time
# 导入rclpy相关库
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
# 导入接口
from robot_control_interfaces.action import MoveRobot
# 导入机器人类
from example_action_rclpy.robot import Robot
#from rclpy.executors import MultiThreadedExecutor
#from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ActionRobot02(Node):
    """机器人端Action服务"""

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")
        
def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    action_robot_02 = ActionRobot02("action_robot_02")
    # 采用多线程执行器解决rate死锁问题
    # executor = MultiThreadedExecutor()
    # executor.add_node(action_robot_02)
    # executor.spin()
    rclpy.spin(action_robot_02)
    rclpy.shutdown()
```

#### 6.1.4 action_control_02.py

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# 导入Action接口
from robot_control_interfaces.action import MoveRobot

class ActionControl02(Node):
    """Action客户端"""

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    action_robot_02 = ActionControl02("action_control_02")
    rclpy.spin(action_robot_02)
    rclpy.shutdown()
```

#### 6.1.5 setup.py

```python
    entry_points={
        'console_scripts': [
            'action_robot_02 = example_action_rclpy.action_robot_02:main',
            'action_control_02 = example_action_rclpy.action_control_02:main'
        ],
    },
```

接着就可以自行编译测试是否可以将节点运行起来了

### 6.2 编写机器人类

```python
class Robot():
    """机器人类，模拟一个机器人"""

    def __init__(self) -> None:
        self.current_pose_ = 0.0
        self.target_pose_ = 0.0
        self.move_distance_ = 0.0
        self.status_ = MoveRobot.Feedback

    def get_status(self):
        """获取状态"""
        return self.status_

    def get_current_pose(self):
        """获取当前位置"""
        return self.current_pose_

    def close_goal(self):
        """接近目标"""
        return math.fabs(self.target_pose_ - self.current_pose_) < 0.01

    def stop_move(self):
        """停止移动"""
        self.status_ = MoveRobot.Feedback.STATUS_STOP

    def move_step(self):
        """移动一小步"""
        direct = self.move_distance_ / math.fabs(self.move_distance_)
        step = direct * math.fabs(self.target_pose_ - self.current_pose_) * 0.1
        self.current_pose_ += step  # 移动一步
        print(f"移动了：{step}当前位置：{self.current_pose_}")
        return self.current_pose_

    def set_goal(self, distance):
        """设置目标"""
        self.move_distance_ = distance
        self.target_pose_ += distance  # 更新目标位置

        if self.close_goal():
            self.stop_move()
            return False

        self.status_ = MoveRobot.Feedback.STATUS_MOVEING  # 更新状态为移动
        return True

```

代码不复杂，就不进行解析了

### 6.3 编写机器人节点

```python
class ActionRobot02(Node):
    """机器人端Action服务"""

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")

        self.robot_ = Robot()

        self.action_server_ = ActionServer(
            self, MoveRobot, 'move_robot', self.execute_callback
            # ,callback_group=MutuallyExclusiveCallbackGroup()
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """执行回调函数,若采用默认handle_goal函数则会自动调用"""
        self.get_logger().info('执行移动机器人')
        feedback_msg = MoveRobot.Feedback()
        self.robot_.set_goal(goal_handle.request.distance)

        # rate = self.create_rate(2)
        while rclpy.ok() and not self.robot_.close_goal():
            # move
            self.robot_.move_step()
            # feedback
            feedback_msg.pose = self.robot_.get_current_pose()
            feedback_msg.status = self.robot_.get_status()
            goal_handle.publish_feedback(feedback_msg)
            # cancel check
            if goal_handle.is_cancel_requested:
                result = MoveRobot.Result()
                result.pose = self.robot_.get_current_pose()
                return result
            # rate.sleep() # Rate会造成死锁，单线程执行器时不能使用
            time.sleep(0.5)

        goal_handle.succeed()
        result = MoveRobot.Result()
        result.pose = self.robot_.get_current_pose()
        return result
```

真是人生苦短，我用Python，这里代码就变得简单了

只指定了一个回调函数` self.execute_callback`，原因在于Python这为我们封装好了几个默认的函数。

打开文件`/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/action/server.py`，查看源码如下

```python
class ActionServer(Waitable):
    """ROS Action server."""

    def __init__(
        self,
        node,
        action_type,
        action_name,
        execute_callback,
        *,
        callback_group=None,
        goal_callback=default_goal_callback,
        handle_accepted_callback=default_handle_accepted_callback,
        cancel_callback=default_cancel_callback,
        goal_service_qos_profile=qos_profile_services_default,
        result_service_qos_profile=qos_profile_services_default,
        cancel_service_qos_profile=qos_profile_services_default,
        feedback_pub_qos_profile=QoSProfile(depth=10),
        status_pub_qos_profile=qos_profile_action_status_default,
        result_timeout=900
    ):
        """
        Create an ActionServer.

        :param node: The ROS node to add the action server to.
        :param action_type: Type of the action.
        :param action_name: Name of the action.
            Used as part of the underlying topic and service names.
        :param execute_callback: Callback function for processing accepted goals.
            This is called if when :class:`ServerGoalHandle.execute()` is called for
            a goal handle that is being tracked by this action server.
        :param callback_group: Callback group to add the action server to.
            If None, then the node's default callback group is used.
        :param goal_callback: Callback function for handling new goal requests.
        :param handle_accepted_callback: Callback function for handling newly accepted goals.
            Passes an instance of `ServerGoalHandle` as an argument.
        :param cancel_callback: Callback function for handling cancel requests.
        :param goal_service_qos_profile: QoS profile for the goal service.
        :param result_service_qos_profile: QoS profile for the result service.
        :param cancel_service_qos_profile: QoS profile for the cancel service.
        :param feedback_pub_qos_profile: QoS profile for the feedback publisher.
        :param status_pub_qos_profile: QoS profile for the status publisher.
        :param result_timeout: How long in seconds a result is kept by the server after a goal
            reaches a terminal state.
        """
```

刚刚说的的几个默认函数

```python
def default_handle_accepted_callback(goal_handle):
    """Execute the goal."""
    goal_handle.execute()


def default_goal_callback(goal_request):
    """Accept all goals."""
    return GoalResponse.ACCEPT


def default_cancel_callback(cancel_request):
    """No cancellations."""
    return CancelResponse.REJECT

```

### 6.4 编写控制节点

```python
class ActionControl02(Node):
    """Action客户端"""

    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")
        self.action_client_ = ActionClient(self, MoveRobot, 'move_robot')
        self.send_goal_timer_ = self.create_timer(1, self.send_goal)

    def send_goal(self):
        """发送目标"""
        self.send_goal_timer_.cancel()
        goal_msg = MoveRobot.Goal()
        goal_msg.distance = 5.0
        self.action_client_.wait_for_server()
        self._send_goal_future = self.action_client_.send_goal_async(goal_msg,
                                                                     feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """收到目标处理结果"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """获取结果反馈"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.pose}')

    def feedback_callback(self, feedback_msg):
        """获取回调反馈"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.pose}')
```

控制节点依然采用三个回调函数实现数据的接收

- `goal_response_callback`，收到目标处理结果。
- `get_result_callback`，获取结果反馈。
- `feedback_callback`，接收过程信息。

### 6.5 构建测试

接着我们可以编译进行测试。

```bash
cd chapt4_ws/
colcon build --packages-up-to example_action_rclpy
# 运行机器人节点
source install/setup.bash 
ros2 run example_action_rclpy  action_robot_02
# 新终端
source install/setup.bash 
ros2 run example_action_rclpy  action_control_02
```

![image-20220615005951649](https://fishros.com/d2lros2/humble/chapt4/get_started/6.%E5%8A%A8%E4%BD%9C%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220615005951649.png)

> 使用Python编写Action，在设计上Python显得比C++更好用些，但背后的逻辑都是一样的。

## 7 通信机制对比总结

### 7.1 话题

话题（Topic）是一种轻量级的通信方式，用于实现发布-订阅模式，即一个节点发布数据，另一个节点订阅数据。话题是一种单向的通信方式，发布者发布数据后，无法获知数据是否被订阅者成功接收。话题的数据类型可以是ROS中定义的任意消息类型。常见的使用话题实现的场景包括传感器数据的传递、节点间的状态信息交换等。

### 7.2 服务

服务是双向的，提供了一种客户端-服务器模式，即客户端向服务器发送请求，服务器响应请求并返回结果。服务可以实现双向通信，并且支持传递任意的ROS消息类型。服务的实现需要定义两个消息类型，一个用于请求，一个用于响应。常见的使用服务实现的场景包括节点之间的命令调用、请求数据等。

### 7.3 参数

参数（Parameter）是ROS 2中节点的一种配置机制，它可以用于对节点进行设置。参数可以存储整数、浮点数、布尔值、字符串等基本类型数据，也可以存储ROS消息类型。参数的读写操作可以通过服务实现。在节点启动时，可以通过ROS参数服务器将参数传递给节点，也可以在运行时动态修改参数。常见的使用参数的场景包括节点的配置、调试等。，原理基于服务。

### 7.4 动作

动作（Action）是ROS 2中的高级通信机制，它可以实现异步的双向通信，并且支持取消、暂停、恢复等操作。动作通常用于需要执行较长时间的任务，如机器人的导航、物体识别等。与服务不同，动作可以通过话题实时发布执行状态、进度等信息，以便客户端监控执行情况。动作的实现需要定义三个消息类型，一个用于请求，一个用于响应，一个用于反馈。常见的使用动作的场景包括机器人的自主导航、物体抓取等。



# 进阶篇——原理进阶

## 1 ROS参数通信原理介绍

ROS2的参数其实是用服务实现的，

随意运行一个节点，你使用下面的指令，就可以看到多出来很多的参数相关的服务。

```bash
ros2 service list
```

比如启动乌龟模拟器

```bash
ros2 run turtlesim turtlesim_node
```

![img](https://fishros.com/d2lros2/humble/chapt4/advanced/1.ROS%E5%8F%82%E6%95%B0%E9%80%9A%E4%BF%A1%E5%8E%9F%E7%90%86/imgs/v2-dd607e84db92b9b8bd955a7a0ae8aab9_720w.webp)

多出来的这些服务就是用于操作这个节点的参数的

```bash
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```

我们如何使用服务查看参数呢？

手动调一下服务就行了~

```bash
ros2 service call /turtlesim/list_parameters rcl_interfaces/srv/ListParameters "{prefixes: [],depth: 0}"
```

![img](https://fishros.com/d2lros2/humble/chapt4/advanced/1.ROS%E5%8F%82%E6%95%B0%E9%80%9A%E4%BF%A1%E5%8E%9F%E7%90%86/imgs/v2-ee1a807a9be19ede27ce2e6b9092cf49_720w.webp)

这里可以看到结果里的四个参数

```bash
names=['background_b', 'background_g', 'background_r','use_sim_time']
```

采用`ros2 param list`再看看对不对

```bash
ros2 param list
```

通过上面的实验告诉我们ROS2的**参数操作其实就是通过服务通信方式实现的**，获取参数列表，set和get操作就是操作相应的服务

# 2 生命周期节点

以前在ROS1中，节点的启动顺序无法被控制，这对整个机器人系统来说是一件非常危险和不可控的事，比如说机器人传感器还未启动就开始进行数据的读取了。

在ROS2中提出了生命周期节点的概念，通过生命周期来控制和检测节点状态。

## 2.1 生命周期节点介绍

ROS2生命周期节点是利用状态机构成的，状态直接的转换依靠ROS2的通信机制完成。

生命周期节点主要有以下几个状态

- 未配置状态（Unconfigured） ，节点开始时的第一个状态，并在出现错误后结束。没有执行，其主要目的是错误恢复。
- 非活跃状态（Inactivate），节点持有资源（发布者、监听者等）和配置（参数、内部变量），但什么也不做。 没有执行，没有传输，传入的数据可以保存在缓冲区中，但不能读取， 主要目的是允许重新配置。
- 活跃状态（Activate）， 正常执行。
- 已完成状态（Finalized），节点已被销毁。

具体的状态之间转换关系请参考下图。

![img](https://fishros.com/d2lros2/humble/chapt4/advanced/3.%E7%94%9F%E5%91%BD%E5%91%A8%E6%9C%9F%E8%8A%82%E7%82%B9%E4%BB%8B%E7%BB%8D/imgs/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L1poYW5nUmVsYXk=,size_16,color_FFFFFF,t_70.png)