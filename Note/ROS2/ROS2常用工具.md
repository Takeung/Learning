# ROS2常用工具

- [ROS2常用工具](#ros2常用工具)
  - [1 启动管理工具-Launch](#1-启动管理工具-launch)
    - [1.1 Launch启动工具介绍](#11-launch启动工具介绍)
      - [1.1.1 问题描述](#111-问题描述)
      - [1.1.2 解决方案](#112-解决方案)
    - [1.2 编写第一个ROS2的launch文件](#12-编写第一个ros2的launch文件)
      - [1.2.1 三种编写launch文件的方法](#121-三种编写launch文件的方法)
      - [1.2.2 使用Python编写Launch](#122-使用python编写launch)
        - [1.2.2.1 创建功能包和launch文件](#1221-创建功能包和launch文件)
        - [1.2.2.2 启动多个节点的示例](#1222-启动多个节点的示例)
        - [1.2.2.3 将launch文件拷贝到安装目录](#1223-将launch文件拷贝到安装目录)
        - [1.2.2.4 编译测试](#1224-编译测试)
    - [1.3 添加参数\&修改命名空间](#13-添加参数修改命名空间)
  - [2 ROS2命令行工具](#2-ros2命令行工具)
    - [2.1 命令小结](#21-命令小结)
  - [3 RVIZ2](#3-rviz2)
    - [3.1 RVIZ2是什么](#31-rviz2是什么)
    - [3.2 RVIZ2 基础配置](#32-rviz2-基础配置)
      - [3.2.1 全局配置](#321-全局配置)
      - [3.2.2 网格](#322-网格)
      - [3.2.3 机器人模型](#323-机器人模型)
      - [3.2.4 TF](#324-tf)
  - [4 RQT工具](#4-rqt工具)
    - [4.1 RQT是什么](#41-rqt是什么)
    - [4.2 体验RQT](#42-体验rqt)
      - [4.2.1 选择插件](#421-选择插件)
      - [4.2.2 插件大观](#422-插件大观)
      - [Introspection / Node Graph](#introspection--node-graph)
      - [Introspection / Process Monitor](#introspection--process-monitor)
      - [Topic/ Message Publisher](#topic-message-publisher)
      - [Service /Service Caller](#service-service-caller)
      - [Visualization / Image View](#visualization--image-view)
      - [Visualization / MatPlot](#visualization--matplot)
      - [Configuration / Parameter Reconfigure](#configuration--parameter-reconfigure)
  - [5 时光记录仪之rosbag2](#5-时光记录仪之rosbag2)
    - [5.1 安装](#51-安装)
    - [5.2 记录](#52-记录)
      - [5.2.1 常用指令](#521-常用指令)
        - [5.2.1.1 记录](#5211-记录)
        - [5.2.1.2 记录多个话题的数据](#5212-记录多个话题的数据)
        - [5.2.1.3 记录所有话题](#5213-记录所有话题)
        - [5.2.1.4 其他选项](#5214-其他选项)
        - [-o name 自定义输出文件的名字](#-o-name-自定义输出文件的名字)
        - [-s 存储格式](#-s-存储格式)
      - [5.2.2 录制chatter](#522-录制chatter)
        - [5.2.2.1 启动talker](#5221-启动talker)
        - [5.2.2.2 录制](#5222-录制)
    - [5.3 查看录制出话题的信息](#53-查看录制出话题的信息)
    - [5.4 播放](#54-播放)
      - [5.4.1 播放话题数据](#541-播放话题数据)
      - [5.4.2 播放选项](#542-播放选项)
        - [5.4.2.1 倍速播放 -r](#5421-倍速播放--r)
        - [5.4.2.2 -l 循环播放](#5422--l-循环播放)
        - [5.4.2.3 播放单个话题](#5423-播放单个话题)
  - [6 兼容仿真工具-Gazebo](#6-兼容仿真工具-gazebo)
    - [6.1 Gazebo VS Rviz2](#61-gazebo-vs-rviz2)
    - [6.2 Gazebo集成ROS2](#62-gazebo集成ros2)
      - [6.2.1 gazebo\_ros\_pkgs](#621-gazebo_ros_pkgs)
    - [6.3 两轮差速小demo](#63-两轮差速小demo)
      - [6.3.1 安装gazebo](#631-安装gazebo)
      - [6.3.2 安装ROS2的两轮差速功能包](#632-安装ros2的两轮差速功能包)
      - [6.3.3 运行两轮差速demo](#633-运行两轮差速demo)
      - [6.3.4 查看话题](#634-查看话题)
      - [6.3.5 让小车前进](#635-让小车前进)
    - [6.4 总结](#64-总结)


> （摘自：[动手学ROS](https://fishros.com/d2lros2/#/humble/chapt3/%E7%AB%A0%E8%8A%82%E5%AF%BC%E8%AF%BB)）

## 1 启动管理工具-Launch

### 1.1 Launch启动工具介绍

#### 1.1.1 问题描述

对于一个机器人系统来说，往往由很多个不同功能的节点组成，启动一个机器人系统时往往需要启动多个节点，同时根据应用场景和机器人的不同，每个节点还会有不同的配置项。

如果每个节点我们都开一个新终端，敲`ros2 run`指令并写一堆参数，这是多么浪费生命且令人绝望的事情。

除了启动，你会发现，一个个关闭也是很难受的。

#### 1.1.2 解决方案

可不可以编写一个类似于脚本的文件来管理节点的启动呢？

ROS2设计时就为我们想好了，为我们设计了一套完整的语法和规则的文件来帮助我们组织节点的启动，这个武器就叫launch文件。

**launch文件允许我们同时启动和配置多个包含ROS 2节点的可执行文件**

> 在ROS1中launch文件只有一种格式以.launch结尾的xml文档，不熟悉的同学写起来被xml语法折磨的死去活来。不过在ROS2中不要担心，因为在ROS2你可以使用Python代码来编写launch文件

### 1.2 编写第一个ROS2的launch文件

#### 1.2.1 三种编写launch文件的方法

ROS2的launch文件有三种格式，python、xml、yaml。其中ROS2官方推荐的时python方式编写launch文件。 原因在于，相较于XML和YAML，**Python是一个编程语言，更加的灵活，我们可以利用Python的很多库来做一些其他工作**（比如创建一些初始化的目录等）。

> 除了灵活还有另外一个原因是ros2/launch（一般launch共功能）和ros2/launch_ros（ROS 2 launch的特性）是用 Python 编写的，我们使用python编写launch文件可以使用XML和YAML中不能用的launch功能。 要说使用python版本的launch有什么坏处，那就是写起来比yaml要冗余。

#### 1.2.2 使用Python编写Launch

我们的目标是编写一个launch文件，最后使用launch指令，同时启动服务端和客户端节点。

##### 1.2.2.1 创建功能包和launch文件

创建文件夹和功能包，接着touch一个launch文件，后缀为`.py`。

```bash
mkdir -p chapt5_ws/src
cd chapt5_ws
ros2 pkg create robot_startup --build-type ament_cmake --destination-directory src
mkdir -p src/robot_startup/launch
touch src/robot_startup/launch/example_action.launch.py
```

##### 1.2.2.2 启动多个节点的示例

我们需要导入两个库，一个叫做`LaunchDescription`，用于对launch文件内容进行描述，一个是Node，用于声明节点所在的位置。

> 注意这里要定一个名字叫做`generate_launch_description`的函数，ROS2会对该函数名字做识别。

```python
# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    action_robot_01 = Node(
        package="example_action_rclcpp",
        executable="action_robot_01"
    )
    action_control_01 = Node(
        package="example_action_rclcpp",
        executable="action_control_01"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [action_robot_01, action_control_01])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
```

##### 1.2.2.3 将launch文件拷贝到安装目录

如果你编写完成后直接编译你会发现`install`目录下根本没有你编写的`launch`文件，后续`launch`自然也找不到这个launch文件。

因为我们用的是`ament_cmake`类型功能包，所以这里要使用cmake命令进行文件的拷贝

```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
```

如果是`ament_python`功能包版

```python
from setuptools import setup
from glob import glob
import os

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    },
)
```

##### 1.2.2.4 编译测试

使用colcon指令编译我们的程序

```bash
colcon build
```

编译完成后，在`chapt5/chapt5_ws/install/robot_startup/share/robot_startup/launch`目录下你应该就可以看到被cmake拷贝过去的launch文件了。

接着运行`

```bash
# source 第四章的工作目录，这样才能找到对应的节点，不信你可以不source试试
source ../chapt4_ws/install/setup.bash
source install/setup.bash
ros2 launch robot_startup example_action.launch.py
# 新终端
ros2 node list #即可看到两个节点
```

![image-20220616135356671](https://fishros.com/d2lros2/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch/imgs/image-20220616135356671.png)

### 1.3 添加参数&修改命名空间

接着我们尝试使用launch运行参数节点，并通过launch传递参数，和给节点以不同的命名空间。

新建`chapt5_ws/src/robot_startup/launch/example_param_rclcpp.launch.py`。

编写内容如下

```python
# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    parameters_basic1 = Node(
        package="example_parameters_rclcpp",
        namespace="rclcpp",
        executable="parameters_basic",
        parameters=[{'rcl_log_level': 40}]
    )
    parameters_basic2 = Node(
        package="example_parameters_rclpy",
        namespace="rclpy",
        executable="parameters_basic",
        parameters=[{'rcl_log_level': 50}]
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [parameters_basic1, parameters_basic2])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
```

编译运行测试

```bash
# source 第四章的工作目录，这样才能找到对应的节点，不信你可以不source试试
source ../chapt4_ws/install/setup.bash
source install/setup.bash
ros2 launch robot_startup example_param_rclcpp.launch.py
# 新终端
ros2 node list #即可看到两个节点
```

![image-20220616140109862](https://fishros.com/d2lros2/humble/chapt5/get_started/1.%E5%90%AF%E5%8A%A8%E7%AE%A1%E7%90%86%E5%B7%A5%E5%85%B7-Launch/imgs/image-20220616140109862.png)

> launch还有很多更深入的用法，可以参考[总结](http://www.robotsfan.com/posts/7a5950c4.html)

## 2 ROS2命令行工具

### 2.1 命令小结

打开终端，输入ros2，你将看到下面的内容：

```bash
usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit

Commands:
  action     Various action related sub-commands
  bag        Various rosbag related sub-commands
  component  Various component related sub-commands
  daemon     Various daemon related sub-commands
  doctor     Check ROS setup and other potential issues
  interface  Show information about ROS interfaces
  launch     Run a launch file
  lifecycle  Various lifecycle related sub-commands
  multicast  Various multicast related sub-commands
  node       Various node related sub-commands
  param      Various param related sub-commands
  pkg        Various package related sub-commands
  run        Run a package specific executable
  security   Various security related sub-commands
  service    Various service related sub-commands
  topic      Various topic related sub-commands
  wtf        Use `wtf` as alias to `doctor`

  Call `ros2 <command> -h` for more detailed usage.
```

每一个`Command`都是对应着ROS2目前所拥有的工具，其实每一个我们在前面的章节中几乎都使用过，而那些没有使用的到的，几乎都是不常用的，所以大家只需要将前面章节中的`CLI`掌握即可。

当我们忘记了某个命令行工具的时候该怎么办，可以使用对应的指令加上-h，即可获取其使用方法。

## 3 RVIZ2

### 3.1 RVIZ2是什么

RVIZ2是ROS2中的一个非常重要且常用的数据可视化工具。

那数据指的是什么数据？可视化又是怎么可视化的？RVIZ2又是如何实现不同数据的可视化的呢？

答案如下：

- 数据：各种调试机器人时常用的数据，比如：图像数据、三维点云数据、地图数据、TF数据，机器人模型数据等等
- 可视化：可视化就是让你直观的看到数据，比如说一个三维的点(100,100,100),通过RVIZ可以将其显示在空间中
- 如何做到不同数据的可视化：强大的插件，如果没有你的数据，你可以自己再写一个插件，即插即用，方便快捷

> 注意：RVIZ强调将数据可视化出来，是已有数据的情况下，把数据显示出来而以，而我们后面要讲的gazebo仿真软件是通过模拟真实环境产生数据，两者用途并不一样。

### 3.2 RVIZ2 基础配置

#### 3.2.1 全局配置

![全局配置](https://fishros.com/d2lros2/humble/chapt5/get_started/3.%E6%95%B0%E6%8D%AE%E5%8F%AF%E8%A7%86%E5%8C%96%E5%B7%A5%E5%85%B7-RVIZ/imgs/6a8c3220a2c643e184269bddcc2eae2b.png)

- Fixed Frame：所有帧参考的帧的名称，坐标都是相对的，这个就是告诉RVIZ你是相对谁的，一般是设置成map或者odom
- Frame Rate：用于设置更新 3D 视图的最大频率。

#### 3.2.2 网格

用于可视化通常与地板平面相关联的网格

![网格](https://fishros.com/d2lros2/humble/chapt5/get_started/3.%E6%95%B0%E6%8D%AE%E5%8F%AF%E8%A7%86%E5%8C%96%E5%B7%A5%E5%85%B7-RVIZ/imgs/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBA6bG86aaZUk9T,size_16,color_FFFFFF,t_70,g_se,x_16.png)

- Reference frame：帧用作网格坐标参考（通常：）
- Plane cell count: 单元格中网格的大小
- Normal cell count：在沿垂直于叶栅平面的网格数（正常：0）
- Cell size：每个网格单元的尺寸（以米为单位）
- Plane：标识网格平面的两个轴

#### 3.2.3 机器人模型

根据 URDF 模型的描述来可视化机器人的模型。

![机器人模型](https://fishros.com/d2lros2/humble/chapt5/get_started/3.%E6%95%B0%E6%8D%AE%E5%8F%AF%E8%A7%86%E5%8C%96%E5%B7%A5%E5%85%B7-RVIZ/imgs/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBA6bG86aaZUk9T,size_20,color_FFFFFF,t_70,g_se,x_16.png)

- Visual enabled: 启用/禁用模型的 3D 可视化
- Description Source：机器人模型文件的来源，可以在File和Topic之间进行选择
- Description Topic: 机器人模型文件所在的话题

#### 3.2.4 TF

可视化构成 TF 广播的所有帧的位置和方向 ![TF](https://fishros.com/d2lros2/humble/chapt5/get_started/3.%E6%95%B0%E6%8D%AE%E5%8F%AF%E8%A7%86%E5%8C%96%E5%B7%A5%E5%85%B7-RVIZ/imgs/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBA6bG86aaZUk9T,size_20,color_FFFFFF,t_70,g_se,x_16.png)

- Marker Scale: 将字和坐标系标识调整的小一些，使其更加可见且不那么混乱
- Update interval：以秒为单位的TF广播更新时间

最佳实践，勾选你想看的Frames，直观的看到两个坐标之间的关系

## 4 RQT工具

rqt_graph这个工具在平时编写ROS2程序中经常使用，除了rqt_graph，ROS2中还有很多非常易用的RQT工具

### 4.1 RQT是什么

RQT是一个GUI框架，通过插件的方式实现了各种各样的界面工具。

强行解读下：RQT就像插座，任何电器只要符合插座的型号就可以插上去工作。

> 说到这里你应该对ROS2的插件化设计感到无比震撼，上节的bag话题录制的存储格式也是插件式的。

### 4.2 体验RQT

没有复杂的指令，一句命令行就可以调出rqt界面。

```bash
rqt
```

打开之后的窗口如下图，空空如也，不要担心，因为我们没有选插件的原因。

![image-20211015151901951](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015151901951.png)

#### 4.2.1 选择插件

这里我们可以选择现有的几个RQT插件来试一试，可以看到和话题、参数、服务、动作四大通信组件相关的工具都有，还有一些可视化、日志和系统计算图等相关的。

![image-20211015151919724](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015151919724.png) 我们按照比较常用的几个来看一下，其他的大家有个印象，后续用到再用。

#### 4.2.2 插件大观

#### Introspection / Node Graph

第一个是肯定是rqt_graph，插件名字叫做Node Graph，用rqt_graph更多的是为了延续ROS1中的用法，这个**插件用于查看节点和节点之间的关系的**。

![image-20211015151937543](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015151937543.png)

#### Introspection / Process Monitor

这个插件可以看到所有与ROS2相关的进程

![image-20211015151949881](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015151949881.png)

#### Topic/ Message Publisher

可以图形化发布话题数据

![image-20211015152002622](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015152002622.png)

#### Service /Service Caller

图形化调用服务工具

![image-20211015152014897](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015152014897.png)

#### Visualization / Image View

看图像话题数据的Image View

![image-20211015152026855](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015152026855.png)

#### Visualization / MatPlot

话题数据图形化工具MqtPlot，可以用这个工具来调PID

![image-20211015152039078](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015152039078.png)

#### Configuration / Parameter Reconfigure

![image-20211015152050157](https://fishros.com/d2lros2/humble/chapt5/get_started/4.%E5%B8%B8%E7%94%A8%E8%B0%83%E8%AF%95%E5%B0%8F%E5%B7%A5%E5%85%B7-RQT/imgs/image-20211015152050157.png)

## 5 时光记录仪之rosbag2

我们就可以使用这个指令将话题数据存储为文件 ，后续我们无需启动节点，直接可以将bag文件里的话题数据发布出来。

> 这个工具在我们做一个真实机器人的时候非常有用，比如我们可以录制一段机器人发生问题的话题数据，录制完成后可以多次发布出来进行测试和实验，也可以将话题数据分享给别人用于验证算法等。

我们尝试使用bag工具来记录话题数据，并二次重放。

### 5.1 安装

当我们安装ROS2的时候，这个命令行工具已经为我们自动安装了，这里我们就不需要再次安装。

### 5.2 记录

#### 5.2.1 常用指令

启动talker

```bash
ros2 run demo_nodes_cpp talker
```

##### 5.2.1.1 记录

`/topic-name` 为话题名字

```bash
ros2 bag record /topic-name
```

##### 5.2.1.2 记录多个话题的数据

```bash
ros2 bag record topic-name1  topic-name2
```

##### 5.2.1.3 记录所有话题

```bash
ros2 bag record -a
```

##### 5.2.1.4 其他选项

##### -o name 自定义输出文件的名字

```bash
ros2 bag record -o file-name topic-name
```

##### -s 存储格式

目前仅支持sqllite3，其他还待拓展。

#### 5.2.2 录制chatter

##### 5.2.2.1 启动talker

运行talker节点

```bash
ros2 run demo_nodes_cpp talker
```

![李四正在发布小说](https://fishros.com/d2lros2/humble/chapt5/get_started/5.%E6%95%B0%E6%8D%AE%E5%BD%95%E6%92%AD%E5%B7%A5%E5%85%B7-rosbag/imgs/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBA6bG86aaZUk9T,size_20,color_FFFFFF,t_70,g_se,x_16.png)

##### 5.2.2.2 录制

接着使用像下面的指令就可以进行话题数据的录制了

```bash
ros2 bag record /chatter
```

如何停止录制呢？我们直接在终端中使用`Ctrl+C`指令打断录制即可

接着你会在终端中发现多处一个文件夹，名字叫做`rosbag2_xxxxxx.db3 `

打开文件夹，可以看到内容

![文件内容](https://fishros.com/d2lros2/humble/chapt5/get_started/5.%E6%95%B0%E6%8D%AE%E5%BD%95%E6%92%AD%E5%B7%A5%E5%85%B7-rosbag/imgs/7d32470a2c12477f8c90a397a9af339a.png)

这样我们就完成了录制。

### 5.3 查看录制出话题的信息

我们在播放一个视频前，可以通过文件信息查看视频的相关信息，比如话题记录的时间，大小，类型，数量

```bash
ros2 bag info bag-file
```

### 5.4 播放

#### 5.4.1 播放话题数据

接着我们就可以重新的播放数据,使用下面的指令可以播放数据

```bash
ros2 bag play xxx.db3
```

使用ros2的topic的指令来查看数据

```bash
ros2 topic echo /chatter
```

#### 5.4.2 播放选项

##### 5.4.2.1 倍速播放 -r

-r选项可以修改播放速率，比如 -r 值，比如 -r 10,就是10倍速，十倍速播放话题

```bash
ros2 bag play rosbag2_2021_10_03-15_31_41_0.db3 -r 10
```

##### 5.4.2.2 -l 循环播放

单曲循环就是它了

```bash
ros2 bag play rosbag2_2021_10_03-15_31_41_0.db3  -l
```

##### 5.4.2.3 播放单个话题

```bash
ros2 bag play rosbag2_2021_10_03-15_31_41_0.db3 --topics /chatter
```

## 6 兼容仿真工具-Gazebo

今天说说Gazebo，有些同学没有学习RVIZ和Gazebo之前，分不清Gazebo和Rviz之间的区别，只道是Gazebo和RVIZ都能显示机器人模型。

### 6.1 Gazebo VS Rviz2

**RVIZ2是用来可视化数据的软件，核心要义是将数据展示出来（我们不生产数据只做数据的搬运工）。 **

**Gazebo是用于模拟真实环境生产数据的（我们不搬运数据只做数据的生产者）**

所以Gazebo可以根据我们所提供的机器人模型文件，传感器配置参数，给机器人创造一个虚拟的环境，虚拟的电机和虚拟的传感器，并通过ROS/ROS2的相关功能包把传感器数据电机数据等发送出来（生产数据）。

这样我们就不用花一分钱，就拥有了各式各样的机器人和传感器（一万八的雷达，也只不过是用鼠标拖拽一下）。

### 6.2 Gazebo集成ROS2

**Gazebo 是一个独立的应用程序，可以独立于ROS或ROS2使用。**

Gazebo与ROS版本的集成是通过一组叫做`gazebo_ros_pkgs`的包 完成的，`gazebo_ros_pkgs`将Gazebo和ROS2连接起来。

![](https://s2.loli.net/2024/07/17/xqZ4T5bHWpOij2t.png)

#### 6.2.1 gazebo_ros_pkgs

gazebo_ros_pkgs不是一个包，是一堆包如下：

- gazebo_dev：开发Gazebo插件可以用的API
- gazebo_msgs：定义的ROS2和Gazebo之间的接口（Topic/Service/Action）
- gazebo_ros：提供方便的 C++ 类和函数，可供其他插件使用，例如转换和测试实用程序。它还提供了一些通常有用的插件。gazebo_ros::Node
- gazebo_plugins：一系列 Gazebo 插件，将传感器和其他功能暴露给 ROS2 例如:
  1. `gazebo_ros_camera` 发布ROS2图像
  2. `gazebo_ros_diff_drive` 通过ROS2控制和获取两轮驱动机器人的接口

> ROS1的插件迁移到ROS2进度：https://github.com/ros-simulation/gazebo_ros_pkgs/wiki

### 6.3 两轮差速小demo

#### 6.3.1 安装gazebo

因为安装ROS2不会默认安装gazebo，所以我们要手动安装，一行命令很简单，如果提示找不到先去更新下ROS2的源。

```bash
sudo apt install gazebo
```

#### 6.3.2 安装ROS2的两轮差速功能包

一行代码全给装了，不差这点空间

```bash
sudo apt install ros-humble-gazebo-*
```

#### 6.3.3 运行两轮差速demo

一行代码搞定

```bash
gazebo /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world 
```

然后你就可以看到一个死丑死丑的小车。

![](https://s2.loli.net/2024/07/17/layT3JzPWFiI921.png)

#### 6.3.4 查看话题

通过下面的指令可看到话题和话题的类型，把目光放到这个话题`/demo/cmd_demo`,下面我们就通过这个话题来控制小车动起来。

```bash
ros2@ros2-TM1613R:~$ ros2 topic list -t
/clock [rosgraph_msgs/msg/Clock]
/demo/cmd_demo [geometry_msgs/msg/Twist]
/demo/odom_demo [nav_msgs/msg/Odometry]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/tf [tf2_msgs/msg/TFMessage]
```

#### 6.3.5 让小车前进

```bash
ros2 topic pub /demo/cmd_demo geometry_msgs/msg/Twist "{linear: {x: 0.2,y: 0,z: 0},angular: {x: 0,y: 0,z: 0}}"
```

然后就可以看到小车动了起来。

![](https://s2.loli.net/2024/07/17/j1UoGNmfxXR5EKc.png)

### 6.4 总结

- RVIZ2是用来可视化数据的软件，核心要义是将数据展示出来（我们不生产数据只做数据的搬运工）
- Gazebo是用于模拟真实环境生产数据的（我们不搬运数据只做数据的生产者)
- Gazebo是独立于ROS/ROS2的软件（还有很多仿真软件可以用ROS/ROS2）
- ROS2和Gazebo之间的桥梁是：gazebo_ros_pkgs