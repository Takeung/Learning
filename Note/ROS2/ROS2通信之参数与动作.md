- [基础篇——中间件与面向对象基础](#基础篇中间件与面向对象基础)
  - [1 从底层理解通信](#1-从底层理解通信)
    - [1.1 通信的目的](#11-通信的目的)
    - [1.2 通信原理](#12-通信原理)
    - [1.3 通信方式](#13-通信方式)
      - [1.3.1 基于TCP/UDP的网络通信方式](#131-基于tcpudp的网络通信方式)
      - [1.3.2 基于共享内存的进程间通信（IPC）方式](#132-基于共享内存的进程间通信ipc方式)
  - [2 通信中间件之ZMQ](#2-通信中间件之zmq)
    - [2.1 ZeroMQ](#21-zeromq)
    - [2.2 PyZmq](#22-pyzmq)
- [入门篇——话题与服务](#入门篇话题与服务)
  - [1 ROS2话题入门](#1-ros2话题入门)
    - [1.1 订阅发布模型](#11-订阅发布模型)
    - [1.2 消息接口](#12-消息接口)
    - [1.3 ROS2话题工具](#13-ros2话题工具)
      - [1.3.1 GUI工具](#131-gui工具)
        - [1.3.1.1 RQT工具之rqt\_graph](#1311-rqt工具之rqt_graph)
      - [1.3.2 CLI工具](#132-cli工具)
        - [1.3.2.1 ros2 topic list 返回系统中当前活动的所有主题的列表](#1321-ros2-topic-list-返回系统中当前活动的所有主题的列表)
        - [1.3.2.2 ros2 topic list -t 增加消息类型](#1322-ros2-topic-list--t-增加消息类型)
        - [1.3.2.3 ros2 topic echo 打印实时话题内容](#1323-ros2-topic-echo-打印实时话题内容)
        - [1.3.2,4 ros2 topic info 查看主题信息](#1324-ros2-topic-info-查看主题信息)
        - [1.3.2.5 ros2 interface show 查看消息类型](#1325-ros2-interface-show-查看消息类型)
        - [1.3.2.6 ros2 topic pub arg 手动发布命令](#1326-ros2-topic-pub-arg-手动发布命令)
  - [2 话题之RCLCPP实现](#2-话题之rclcpp实现)
    - [2.1 创建节点](#21-创建节点)
    - [2.2 编写发布者](#22-编写发布者)
      - [2.2.1 学习使用API文档](#221-学习使用api文档)
      - [2.2.2 导入消息接口](#222-导入消息接口)
      - [2.2.3 创建发布者](#223-创建发布者)
      - [2.2.4 使用定时器定时发布数据](#224-使用定时器定时发布数据)
        - [2.2.4.1 查看定时器API](#2241-查看定时器api)
        - [2.2.4.2 编写代码](#2242-编写代码)
        - [2.2.4.3 代码讲解](#2243-代码讲解)
      - [2.2.5 运行测试](#225-运行测试)
    - [2.3 编写订阅者](#23-编写订阅者)
      - [2.3.1 创建订阅节点](#231-创建订阅节点)
      - [2.3.2 查看订阅者API文档](#232-查看订阅者api文档)
      - [2.3.3 编写代码](#233-编写代码)
      - [2.3.4 运行测试](#234-运行测试)
    - [2.4 订阅发布测试](#24-订阅发布测试)
  - [3 话题之RCLPY实现](#3-话题之rclpy实现)
    - [3.1 创建功能包和节点](#31-创建功能包和节点)
    - [3.2 编写订阅者](#32-编写订阅者)
    - [3.3 编写发布者](#33-编写发布者)
    - [3.4 运行测试](#34-运行测试)
      - [3.4.1 发布节点](#341-发布节点)
      - [3.4.2 订阅节点](#342-订阅节点)
      - [3.4.3 RQT](#343-rqt)
  - [4 ROS2服务入门](#4-ros2服务入门)
    - [4.1 服务通信介绍](#41-服务通信介绍)
    - [4.2 体验服务](#42-体验服务)
      - [4.2.1 启动服务端](#421-启动服务端)
      - [4.2.2 使用命令查看服务列表](#422-使用命令查看服务列表)
      - [4.2.3 手动调用服务](#423-手动调用服务)
    - [4.3 ROS2服务常用命令](#43-ros2服务常用命令)
      - [4.3.1 查看服务列表](#431-查看服务列表)
      - [4.3.2 手动调用服务](#432-手动调用服务)
      - [4.3.3 查看服务接口类型](#433-查看服务接口类型)
      - [4.3.4 查找使用某一接口的服务](#434-查找使用某一接口的服务)
  - [5 服务之RCLCPP实现](#5-服务之rclcpp实现)
    - [5.1 创建功能包和节点](#51-创建功能包和节点)
    - [5.2 服务端实现](#52-服务端实现)
      - [5.2.1 导入接口](#521-导入接口)
      - [5.2.2 编写代码](#522-编写代码)
      - [5.2.3 测试](#523-测试)
    - [5.3 客户端实现](#53-客户端实现)
      - [5.3.1 API接口](#531-api接口)
        - [5.3.1.1 create\_client](#5311-create_client)
        - [5.3.1.2 async\_send\_request](#5312-async_send_request)
        - [5.3.1.3 wait\_for\_service](#5313-wait_for_service)
      - [5.3.2 代码](#532-代码)
      - [5.3.3 测试](#533-测试)
  - [6 服务之RCLPY实现](#6-服务之rclpy实现)
    - [6.1 创建功能包和节点](#61-创建功能包和节点)
    - [6.2 服务端实现](#62-服务端实现)
      - [6.2.1 看 API](#621-看-api)
      - [6.2.2 写代码](#622-写代码)
      - [6.2.3 测试](#623-测试)
    - [6.3 客户端实现](#63-客户端实现)
      - [6.3.1 API](#631-api)
      - [6.3.2 写代码](#632-写代码)
      - [6.3.3 测试](#633-测试)
  - [7 ROS2接口介绍](#7-ros2接口介绍)
    - [7.1 接口](#71-接口)
      - [7.1.1 什么是接口](#711-什么是接口)
      - [7.1.2 为什么使用接口](#712-为什么使用接口)
      - [7.1.3 ROS2自带的接口](#713-ros2自带的接口)
    - [7.2 接口文件内容](#72-接口文件内容)
      - [7.2.1 可以定义的接口三种类型](#721-可以定义的接口三种类型)
      - [7.2.2 接口形式](#722-接口形式)
      - [7.2.3 接口数据类型](#723-接口数据类型)
      - [7.2.4 接口如何生成代码](#724-接口如何生成代码)
    - [7.3 自定义接口实践](#73-自定义接口实践)
      - [7.3.1 场景定义](#731-场景定义)
      - [7.3.2 定义接口](#732-定义接口)
      - [7.3.3 创建接口功能包编接口](#733-创建接口功能包编接口)
    - [7.4 ROS2接口常用CLI命令](#74-ros2接口常用cli命令)
      - [7.4.1 查看接口列表](#741-查看接口列表)
      - [7.4.2 查看某一个接口详细的内容](#742-查看某一个接口详细的内容)
  - [8 自定义接口RCLCPP实战](#8-自定义接口rclcpp实战)
    - [8.1 创建功能包和节点](#81-创建功能包和节点)
    - [8.2 编写机器人类](#82-编写机器人类)
    - [8.3 编写机器人节点逻辑](#83-编写机器人节点逻辑)
    - [8.4 编写控制节点](#84-编写控制节点)
    - [8.5 测试运行](#85-测试运行)
      - [8.5.1 编译](#851-编译)
      - [8.5.2 测试](#852-测试)
      - [8.5.3 思考](#853-思考)
  - [9 自定义接口RCLPY实战](#9-自定义接口rclpy实战)
    - [9.1 创建功能包](#91-创建功能包)
    - [9.2 编写机器人类](#92-编写机器人类)
    - [9.3 编写机器人节点](#93-编写机器人节点)
    - [9.4 编写控制节点](#94-编写控制节点)
    - [9.5 运行测试](#95-运行测试)
- [进阶篇——中间件进阶](#进阶篇中间件进阶)
  - [1 原始数据类型与包装类型](#1-原始数据类型与包装类型)
    - [1.1 数据名称](#11-数据名称)
    - [1.2 数据类型](#12-数据类型)
      - [1.2.1 数据类型有哪些呢？](#121-数据类型有哪些呢)
      - [1.2.2 类型扩展（套娃）](#122-类型扩展套娃)
        - [1.2.2.1 第一层套娃](#1221-第一层套娃)
        - [1.2.2.2 第二层套娃](#1222-第二层套娃)
        - [1.2.2.3 第三层套娃](#1223-第三层套娃)
    - [1.3 接口类型总结](#13-接口类型总结)
  - [2 通信质量Qos配置指南](#2-通信质量qos配置指南)
  - [3 DDS进阶之Fast-DDS环境搭建](#3-dds进阶之fast-dds环境搭建)
    - [3.1 论FastDDS的三种打开方式](#31-论fastdds的三种打开方式)
    - [3.2 源码编译安装FastDDS](#32-源码编译安装fastdds)
      - [3.2.1 安装工具和依赖库](#321-安装工具和依赖库)
      - [3.2.2 创建目录，下载仓库](#322-创建目录下载仓库)
      - [3.2.3 安装gradle 6.4](#323-安装gradle-64)
      - [3.2.4 编译](#324-编译)
      - [3.2.5 最后一步:配置环境变量](#325-最后一步配置环境变量)
  - [4 使用DDS进行订阅发布](#4-使用dds进行订阅发布)
    - [4.1 HelloFish例程](#41-hellofish例程)
      - [4.1.1 下载代码](#411-下载代码)
      - [4.1.2 编译例程](#412-编译例程)
      - [4.1.3 执行例程](#413-执行例程)
      - [4.1.4 查看结果](#414-查看结果)


（摘自：[动手学ROS](https://fishros.com/d2lros2/#/humble/chapt3/%E7%AB%A0%E8%8A%82%E5%AF%BC%E8%AF%BB)）

# 基础篇——中间件与面向对象基础

## 1 从底层理解通信

当涉及到底层通信时，通过在Linux命令行中执行一些相关操作可以更好地理解通信的工作方式，1.3节提供了一些Linux命令行来说明通信方式的示例：

### 1.1 通信的目的

通信的目的是在计算机系统中实现不同组件、进程或设备之间的信息和数据传递。通过通信，各个实体可以共享信息、协调行动并实现协同工作。在计算机领域，通信是构建分布式系统、网络和协议的基础。

### 1.2 通信原理

通信的原理涉及两个主要方面：**通信协议**和**通信方式**。通信协议定义了数据的格式、传输方式、错误检测和纠正等规则，以确保可靠的数据传输。通信方式涉及了不同的通信介质和技术，包括网络通信和进程间通信（IPC）。

### 1.3 通信方式

#### 1.3.1 基于TCP/UDP的网络通信方式

基于TCP/UDP的网络通信方式通过计算机网络进行信息交换。其中，TCP（传输控制协议）提供可靠的、面向连接的通信，而UDP（用户数据报协议）则是无连接的通信方式。在Linux命令行中，可以使用诸如`ping`和`nc`命令来演示网络通信。

例如，使用`ping`命令进行基于UDP的网络通信：

```bash
ping 192.168.0.1
```

该命令将向IP地址为192.168.0.1的主机发送ICMP Echo请求，并等待接收相应的回复。

使用`nc`命令进行基于TCP的网络通信：

```bash
nc -l 1234
```

该命令将在本地监听端口1234，并等待与之建立TCP连接的客户端。通过在另一个终端窗口中执行以下命令，可以建立与本地1234端口的TCP连接并在连接上发送消息：

```bash
echo "Hello, TCP!" | nc 127.0.0.1 1234
```

#### 1.3.2 基于共享内存的进程间通信（IPC）方式

基于共享内存的IPC方式通过共享内存区域在同一计算机系统内的不同进程之间进行通信。在Linux命令行中，可以使用`ipcs`和`ipcrm`命令来管理共享内存段。

通过`ipcs`命令查看当前系统中的共享内存段：

```bash
ipcs -m
```

使用`ipcrm`命令删除不再需要的共享内存段：

```bash
ipcrm -m <shmid>
```

通过以上示例，我们可以更好地理解通信的目的和原理，并使用Linux命令行演示了基于TCP/UDP的网络通信和基于共享内存的IPC通信的示例。这有助于进一步理解通信的实际应用和操作。

## 2 通信中间件之ZMQ

### 2.1 [ZeroMQ](https://zeromq.org/)

众所周知，FastDDS是ROS2的通信中间件，而FastDDS比ZeroMQ性能好，原因是ZeroMQ非常的轻量，也就是小巧，占用资源少，看名字，Zero Message Queue，零消息队列。ZeroMQ提供了各种（如进程内、进程间、TCP 和多播）消息传输的套接字。

### 2.2 [PyZmq](https://pyzmq.readthedocs.io/en/latest/)

PyZmq也提供了类似于订阅发布的方式来传递消息，还有更多的使用方法，比如客户端服务端这种，网上有[大佬](https://www.php.cn/python-tutorials-459626.html)已经探索了。



# 入门篇——话题与服务

## 1 ROS2话题入门

话题是ROS2中最常用的通信方式之一，话题通信采取的是**订阅发布模型**。

> 参考链接：[Understanding ROS 2 topics — ROS 2 Documentation: Humble documentation](http://docs.ros.org/en/humble/Tutorials/Topics/Understanding-ROS2-Topics.html)

### 1.1 订阅发布模型

一个节点发布数据到某个话题上，另外一个节点就可以通过订阅话题拿到数据。

![](https://s2.loli.net/2024/07/15/9UtFuPkyvOJfXSi.png)

除了上述这种一个节点发布，一个节点接受的形式外，ROS2话题通信其实还可以是`1对n`,`n对1`,`n对n`的。

1对n

![](https://s2.loli.net/2024/07/15/2rXcGSxQLUqutR9.png)

n对1（同一个话题可以有多个发布者）

![](https://s2.loli.net/2024/07/15/Qz5oiaywZRfpM2q.png)

n对n

![](https://s2.loli.net/2024/07/15/Yzwar6CyhUm9d3V.png)

还有一种就是ROS2节点可以订阅本身发布的话题

![](https://s2.loli.net/2024/07/15/aipXLMqe4GOYyKw.png)

### 1.2 消息接口

为了方便发送者和接收者进行数据的交换，ROS2帮我们在数据传递时做好了消息的序列化和反序列化（有关消息序列化相关内容请参考本章基础篇），而且ROS2的消息序列化与反序列化通信是可以做到跨编程语言、跨平台和跨设备之间的。

ROS2如何做到跨编程语言、跨平台和跨设备之间的数据收发呢？这就得益于通过定义消息接口文件了。

简而言之，当我们定义好消息接口后，ROS2会根据消息接口内容生成不同语言的接口类，在不同编程语言中调用相同的类即可实现无感的消息序列化和反序列化。

消息接口需要遵循这样一条规则：**同一个话题，所有的发布者和接收者必须使用相同消息接口**。

### 1.3 ROS2话题工具

#### 1.3.1 GUI工具

##### 1.3.1.1 RQT工具之rqt_graph

ROS2作为一个强大的工具，在运行过程中，我们是可以通过命令来看到节点和节点之间的数据关系的。

运行demo_nodes，依次打开三个终端，分别输入下面三个命令。

```bash
ros2 run demo_nodes_py listener
ros2 run demo_nodes_cpp talker
rqt_graph
```

你将看到下面这张图

![img](https://fishros.com/d2lros2/humble/chapt3/get_started/1.ROS2%E8%AF%9D%E9%A2%98%E5%85%A5%E9%97%A8/imgs/image-20210803113450234.png)

> 你可以尝试改变菜单栏的Hide或者Group选项，看一看下面图的变化，感受一下rqt_graph工具的强大。

这是一个很重要的工具，在学习和使用ROS2的过程中会经常用到，来看一看数据到底是怎么走的，它可以帮我们搞清楚一个节点的输入和输出是什么。

#### 1.3.2 CLI工具

ros2也支持很多强大的topic指令。可以使用下面的指令查看。

```bash
ros2 topic -h
```

![image-20210803114102048](https://fishros.com/d2lros2/humble/chapt3/get_started/1.ROS2%E8%AF%9D%E9%A2%98%E5%85%A5%E9%97%A8/imgs/image-20210803114102048.png)

下面先介绍几个比较常用的命令

##### 1.3.2.1 ros2 topic list 返回系统中当前活动的所有主题的列表

命令

```bash
ros2 topic list
```

结果

![image-20210803114705943](https://fishros.com/d2lros2/humble/chapt3/get_started/1.ROS2%E8%AF%9D%E9%A2%98%E5%85%A5%E9%97%A8/imgs/image-20210803114705943.png)

##### 1.3.2.2 ros2 topic list -t 增加消息类型

命令

```bash
ros2 topic list -t
```

结果

![image-20210803114756448](https://fishros.com/d2lros2/humble/chapt3/get_started/1.ROS2%E8%AF%9D%E9%A2%98%E5%85%A5%E9%97%A8/imgs/image-20210803114756448.png)

##### 1.3.2.3 ros2 topic echo 打印实时话题内容

命令

```bash
ros2 topic echo /chatter
```

结果

![image-20210803115124591](https://fishros.com/d2lros2/humble/chapt3/get_started/1.ROS2%E8%AF%9D%E9%A2%98%E5%85%A5%E9%97%A8/imgs/image-20210803115124591.png)

##### 1.3.2,4 ros2 topic info 查看主题信息

命令

```bash
ros2 topic info /chatter
```

结果

![image-20210803115320265](https://fishros.com/d2lros2/humble/chapt3/get_started/1.ROS2%E8%AF%9D%E9%A2%98%E5%85%A5%E9%97%A8/imgs/image-20210803115320265.png)

##### 1.3.2.5 ros2 interface show 查看消息类型

上面一个指令告诉大家这个消息是std_msgs/msg/String，那String里面有什么呢？不妨来试一试。

命令

```bash
ros2 interface show std_msgs/msg/String
```

结果

![image-20210803115726942](https://fishros.com/d2lros2/humble/chapt3/get_started/1.ROS2%E8%AF%9D%E9%A2%98%E5%85%A5%E9%97%A8/imgs/image-20210803115726942.png)

##### 1.3.2.6 ros2 topic pub arg 手动发布命令

关闭发布者，我们手动发布

命令

```bash
ros2 topic pub /chatter std_msgs/msg/String 'data: "123"'
```

结果

![image-20210803115906116](https://fishros.com/d2lros2/humble/chapt3/get_started/1.ROS2%E8%AF%9D%E9%A2%98%E5%85%A5%E9%97%A8/imgs/image-20210803115906116.png)

## 2 话题之RCLCPP实现

RCLCPP为Node类提供了丰富的API接口，其中就包括创建话题发布者和创建话题订阅者。

### 2.1 创建节点

首先创建一个`控制节点`和一个`被控节点`。

控制节点创建一个话题发布者，发布控制命令（command）话题，接口类型为字符串（string），控制接点通过发布者发布控制命令（前进、后退、左转、右转、停止）。

被控节点创建一个订阅者，订阅控制命令，收到控制命令后根据命令内容打印对应速度出来。

![](https://s2.loli.net/2024/07/15/NUaOfQcZEV6qwYz.png)

依次输入下面的命令，创建`chapt3_ws`工作空间、`example_topic_rclcpp`功能包和`topic_publisher_01.cpp`。

```bash
cd d2lros2/
mkdir -p chapt3/chapt3_ws/src
cd chapt3/chapt3_ws/src
ros2 pkg create example_topic_rclcpp --build-type ament_cmake --dependencies rclcpp
touch example_topic_rclcpp/src/topic_publisher_01.cpp
```

完成后目录结构

```bash
.
└── src
    └── example_topic_rclcpp
        ├── CMakeLists.txt
        ├── include
        │   └── example_topic_rclcpp
        ├── package.xml
        └── src
            └── topic_publisher_01.cpp

5 directories, 3 files
```

接着采用面向对象方式写一个最简单的节点。

```cpp
#include "rclcpp/rclcpp.hpp"

class TopicPublisher01 : public rclcpp::Node
{
  public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
    }

  private:
    // 声明节点
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

修改CMakeLists.txt

```cmake
add_executable(topic_publisher_01 src/topic_publisher_01.cpp)
ament_target_dependencies(topic_publisher_01 rclcpp)

install(TARGETS
  topic_publisher_01
  DESTINATION lib/${PROJECT_NAME}
)
```

接着可以编译测试下，注意运行colcon的目录。

```bash
cd chapt3/chapt3_ws/
colcon build --packages-select example_topic_rclcpp
source install/setup.bash
ros2 run example_topic_rclcpp topic_publisher_01
```

![image-20220604202219516](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220604202219516.png)

### 2.2 编写发布者

#### 2.2.1 学习使用API文档

想要创建发布者，只需要调用`node`的成员函数`create_publisher`并传入对应的参数即可。

有关API文档详细内容可以访问：[rclcpp: rclcpp: ROS Client Library for C++ (ros2.org)](https://docs.ros2.org/latest/api/rclcpp/)

![image-20220604203349982](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220604203349982.png)

打开主页，可以看到创建发布者的函数，进去即可看到参数和详细解释。

![image-20220604203512857](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220604203512857.png)

通过文档可以看出，我们至少需要传入消息类型（msgT）、话题名称（topic_name）和 服务指令（qos）。

#### 2.2.2 导入消息接口

消息接口是ROS2通信时必须的一部分，通过消息接口ROS2才能完成消息的序列化和反序列化。ROS2为我们定义好了常用的消息接口，并生成了C++和Python的依赖文件，我们可以直接在程序中进行导入。

`ament_cmake`类型功能包导入消息接口分为三步：

1. 在`CMakeLists.txt`中导入，具体是先`find_packages`再`ament_target_dependencies`。
2. 在`packages.xml`中导入，具体是添加`depend`标签并将消息接口写入。
3. 在代码中导入，C++中是`#include"消息功能包/xxx/xxx.hpp"`。

我们依次做完这三步后文件内容如下：

`CMakeLists.txt`

```camke
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(topic_publisher_01 src/topic_publisher_01.cpp)
ament_target_dependencies(topic_publisher_01 rclcpp std_msgs)
```

`packages.xml`

```xml
  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
```

代码文件`topic_publisher_01.cpp`

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicPublisher01 : public rclcpp::Node
```

#### 2.2.3 创建发布者

根据ROS2的RCLCPPAPI文档可以看出，我们需要提供消息接口、话题名称和服务质量Qos。

- 消息接口上面我们已经导入了，是`std_msgs/msg/string.h`。
- 话题名称（topic_name），我们就用`control_command`。
- Qos，Qos支持直接指定一个数字，这个数字对应的是`KeepLast`队列长度。一般设置成10，即如果一次性有100条消息，默认保留最新的10个，其余的都扔掉。

接着我们可以编写发布者的代码了。

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicPublisher01 : public rclcpp::Node
{
  public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
    }

  private:
    // 声明话题发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
};
```

#### 2.2.4 使用定时器定时发布数据

##### 2.2.4.1 查看定时器API

虽然编写好了发布者，但是我们还没有发布数据，我们需要通过ROS2中的定时器来设置指定的周期调用回调函数，在回调函数里实现发布数据功能。

> 有关回调函数和定时器相关内容请参考基础篇多线程与回调函数相关内容。

再次找到RCLCPP文档，找到创建定时器函数，观察参数

![image-20220605145907843](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220605145907843.png)

- period，回调函数调用周期。
- callback，回调函数。
- group，调用回调函数所在的回调组，默认为nullptr。

##### 2.2.4.2 编写代码

```c++
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicPublisher01 : public rclcpp::Node
{
  public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        // 创建消息
        std_msgs::msg::String message;
        message.data = "forward";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
        command_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
};
```

##### 2.2.4.3 代码讲解

**定时器**

定时器是ROS2中的另外一个常用功能，通过定时器可以实现按照一定周期调用某个函数以实现定时发布等逻辑。

定时器对应的类是` rclcpp::TimerBase`，调用`create_wall_timer`将返回其共享指针。

创建定时器时传入了两个参数，这两个参数都利用了C++11的新特性。

- `std::chrono::milliseconds(500)`，代表500ms，chrono是c++ 11中的时间库，提供计时，时钟等功能。
- `std::bind(&TopicPublisher01::timer_callback, this)`，bind() 函数的意义就像它的函数名一样，是用来绑定函数调用的某些参数的。

**创建发布消息**

`std_msgs::msg::String`是通过ROS2的消息文件自动生成的类，其原始消息文件内容可以通过命令行查询

```bash
ros2 interface show std_msgs/msg/String 
```

结果

```bash
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data
```

可以看到其内部包含了一个`string data`，**ROS2会将消息文件转换成一个类，并把其中的定义转换成类的成员函数。**

#### 2.2.5 运行测试

编译，source，运行

```bash
cd chapt3/chapt3_ws/
colcon build --packages-select example_topic_rclcpp
source install/setup.bash
ros2 run example_topic_rclcpp topic_publisher_01
```

测试

```bash
# 查看列表
ros2 topic list
# 输出内容
ros2 topic echo /command
```

![image-20220605155135956](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220605155135956.png)

### 2.3 编写订阅者

之所以我们可以用命令行看到数据，原因在于CLI创建了一个订阅者来订阅`/command`指令。接下来我们将要手动创建一个节点订阅并处理数据。

#### 2.3.1 创建订阅节点

```bash
cd chapt3_ws/src/example_topic_rclcpp
touch src/topic_subscribe_01.cpp
```

编写代码

```c++
#include "rclcpp/rclcpp.hpp"

class TopicSubscribe01 : public rclcpp::Node
{
  public:
    // 构造函数,有一个参数为节点名称
    TopicSubscribe01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
    }

  private:
    // 声明节点
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicSubscribe01>("topic_subscribe_01");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

CMakeLists.txt

```cmake
add_executable(topic_subscribe_01 src/topic_subscribe_01.cpp)
ament_target_dependencies(topic_subscribe_01 rclcpp)

install(TARGETS
topic_subscribe_01
  DESTINATION lib/${PROJECT_NAME}
)
```

编译测试

```bash
cd chapt3/chapt3_ws/
colcon build --packages-select example_topic_rclcpp
source install/setup.bash
ros2 run example_topic_rclcpp topic_subscribe_01
```

![image-20220605165400876](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220605165400876.png)

#### 2.3.2 查看订阅者API文档

看API文档或者看头文件中关于函数的定义这是一个好习惯。

![image-20220605164019985](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220605164019985.png)

五个参数，但后面两个都是默认的参数，我们只需要有话题名称、Qos和回调函数即可。

#### 2.3.3 编写代码

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicSubscribe01 : public rclcpp::Node
{
  public:
    TopicSubscribe01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
          // 创建一个订阅者订阅话题
        command_subscribe_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&TopicSubscribe01::command_callback, this, std::placeholders::_1));
    }

  private:
     // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscribe_;
     // 收到话题数据的回调函数
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        double speed = 0.0f;
        if(msg->data == "forward")
        {
            speed = 0.2f;
        }
        RCLCPP_INFO(this->get_logger(), "收到[%s]指令，发送速度 %f", msg->data.c_str(),speed);
    }
};
```

依然的需要在`CMakeLists.txt`添加下`std_msgs`依赖

```cmake
ament_target_dependencies(topic_subscribe_01 rclcpp std_msgs)
```

`packages.xml`就不需要了，同一个功能包，已经添加了。

#### 2.3.4 运行测试

编译运行订阅节点

```bash
cd chapt3/chapt3_ws/
colcon build --packages-select example_topic_rclcpp
source install/setup.bash
ros2 run example_topic_rclcpp topic_subscribe_01
```

手动发布数据测试订阅者

```bash
ros2 topic pub /command std_msgs/msg/String "{data: forward}"
```

![image-20220605170254540](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220605170254540.png)

### 2.4 订阅发布测试

关闭上面启动的终端，重新运行指令

```bash
cd chapt3/chapt3_ws/
source install/setup.bash
ros2 run example_topic_rclcpp topic_subscribe_01
```

运行发布节点

```bash
cd chapt3/chapt3_ws/
source install/setup.bash
ros2 run example_topic_rclcpp topic_publisher_01
```

运行结果

![image-20220605165753271](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220605165753271.png)

查看计算图`rqt`

打开RQT、选择Node Graph、点击![image-20220605170035287](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220605170035287.png)刷新下

![image-20220605170006221](https://fishros.com/d2lros2/humble/chapt3/get_started/2.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220605170006221.png)

## 3 话题之RCLPY实现

有了前面的经验，实现Python版本的发布订阅也非常的轻松了，因为ROS2的API一致性保持的很好，这点值得点赞。

- [Node — rclpy 0.6.1 documentation (ros2.org)](https://docs.ros2.org/latest/api/rclpy/api/node.html)

### 3.1 创建功能包和节点

创建功能包

```bash
cd chapt3/chapt3_ws/src/
ros2 pkg create example_topic_rclpy  --build-type ament_python --dependencies rclpy
```

创建节点文件

```bash
cd example_topic_rclpy/example_topic_rclpy
touch topic_subscribe_02.py
touch topic_publisher_02.py
```

简单编写下代码，依然采用类的形式

发布者

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NodePublisher02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodePublisher02("topic_publisher_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

订阅节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NodeSubscribe02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeSubscribe02("topic_subscribe_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

setup.py

```python
    entry_points={
        'console_scripts': [
            "topic_publisher_02 = example_topic_rclpy.topic_publisher_02:main",
            "topic_subscribe_02 = example_topic_rclpy.topic_subscribe_02:main"
        ],
    },
```

### 3.2 编写订阅者

```python
from std_msgs.msg import String

class NodeSubscribe02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        # 创建订阅者
        self.command_subscribe_ = self.create_subscription(String,"command",self.command_callback,10)

    def command_callback(self,msg):
        speed = 0.0
        if msg.data=="backup":
            speed = -0.2
        self.get_logger().info(f'收到[{msg.data}]命令，发送速度{speed}')
```

### 3.3 编写发布者

```python
from std_msgs.msg import String

class NodePublisher02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.command_publisher_ = self.create_publisher(String,"command", 10) 
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        """
        定时器回调函数
        """
        msg = String()
        msg.data = 'backup'
        self.command_publisher_.publish(msg) 
        self.get_logger().info(f'发布了指令：{msg.data}')    #打印一下发布的数据
```

### 3.4 运行测试

#### 3.4.1 发布节点

```bash
cd chapt3/chapt3_ws/
source install/setup.bash
ros2 run example_topic_rclpy topic_publisher_02
```

#### 3.4.2 订阅节点

```bash
cd chapt3/chapt3_ws/
source install/setup.bash
ros2 run example_topic_rclpy topic_subscribe_02
```

[![image-20220605174740555](https://fishros.com/d2lros2/humble/chapt3/get_started/3.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220605174740555.png)](https://fishros.com/d2lros2/#/humble/chapt3/get_started/3.话题之RCLPY实现?id)

#### 3.4.3 RQT

![image-20220605175201183](https://fishros.com/d2lros2/humble/chapt3/get_started/3.%E8%AF%9D%E9%A2%98%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220605175201183.png)

## 4 ROS2服务入门

### 4.1 服务通信介绍

服务分为客户端和服务端，平时我们用的手机APP都可以成为客户端，而APP服务器对于软件来说就是服务端。

客户端发送请求给服务端，服务端可以根据客户端的请求做一些处理，然后返回结果给客户端。

发送响应发送请求服务端客户端

所以服务-客户端模型，也可以成为请求-响应模型。

> 不知道你有没有感觉到服务和话题的不同之处，**话题是没有返回**的，适用于单向或大量的数据传递。而**服务是双向的**，客户端发送请求，服务端响应请求。

同时服务还是有一些注意事项：

- 同一个服务（名称相同）有且只能有一个节点来提供
- 同一个服务可以被多个客户端调用

放两张官方形象的动图：

![img](https://fishros.com/d2lros2/humble/chapt3/get_started/4.ROS2%E6%9C%8D%E5%8A%A1%E5%85%A5%E9%97%A8/imgs/Service-SingleServiceClient.gif)

![img](https://fishros.com/d2lros2/humble/chapt3/get_started/4.ROS2%E6%9C%8D%E5%8A%A1%E5%85%A5%E9%97%A8/imgs/Service-MultipleServiceClient.gif)

### 4.2 体验服务

在我们安装ROS2的时候其实系统为我们安装了一些样例程序，其中就有服务使用样例，我们可以先来体验一下。

#### 4.2.1 启动服务端

打开终端，运行下面的命令，这个命令用于运行一个服务节点，这个服务的功能是将两个数字相加，给定a，b两个数，返回sum也就是ab之和。

```bash
ros2 run examples_rclpy_minimal_service service
```

#### 4.2.2 使用命令查看服务列表

```bash
ros2 service list
```

![image-20210810104048993](https://fishros.com/d2lros2/humble/chapt3/get_started/4.ROS2%E6%9C%8D%E5%8A%A1%E5%85%A5%E9%97%A8/imgs/image-20210810104048993.png)

#### 4.2.3 手动调用服务

再启动一个终端，输入下面的命令（注意a：、b：后的空格）。

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"
```

![image-20210810113831471](https://fishros.com/d2lros2/humble/chapt3/get_started/4.ROS2%E6%9C%8D%E5%8A%A1%E5%85%A5%E9%97%A8/imgs/image-20210810113831471.png)

我们可以看到客户端请求两个数字5+10，服务端返回15。

### 4.3 ROS2服务常用命令

ROS2的命令行工具，小鱼觉得还是非常值得一学的，毕竟确实很实用（装X），之前已经给大家讲过了关于节点、话题、接口相关的命令了，现在小鱼说一下关于服务的那些命令行。

#### 4.3.1 查看服务列表

```bash
ros2 service list
```

![image-20210810115216800](https://fishros.com/d2lros2/humble/chapt3/get_started/4.ROS2%E6%9C%8D%E5%8A%A1%E5%85%A5%E9%97%A8/imgs/image-20210810115216800.png)

#### 4.3.2 手动调用服务

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"
```

![image-20210810115316799](https://fishros.com/d2lros2/humble/chapt3/get_started/4.ROS2%E6%9C%8D%E5%8A%A1%E5%85%A5%E9%97%A8/imgs/image-20210810115316799.png)

如果不写参数值调用会怎么样？比如下面这种，大家可以尝试下。

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts
```

> 不写参数则是会以默认参数计算并返回，此处默认返回0。

#### 4.3.3 查看服务接口类型

```bash
ros2 service type /add_two_ints
```

![image-20210810115428267](https://fishros.com/d2lros2/humble/chapt3/get_started/4.ROS2%E6%9C%8D%E5%8A%A1%E5%85%A5%E9%97%A8/imgs/image-20210810115428267.png)

#### 4.3.4 查找使用某一接口的服务

这个命令看起来和4.3.3刚好相反。

```bash
ros2 service find example_interfaces/srv/AddTwoInts
```

![image-20210810115552147](https://fishros.com/d2lros2/humble/chapt3/get_started/4.ROS2%E6%9C%8D%E5%8A%A1%E5%85%A5%E9%97%A8/imgs/image-20210810115552147.png)

## 5 服务之RCLCPP实现

利用rclcpp提供的示例接口实现两数相加的服务端和客户端。

### 5.1 创建功能包和节点

```bash
cd chapt3/chapt3_ws/src
ros2 pkg create example_service_rclcpp --build-type ament_cmake --dependencies rclcpp
touch example_service_rclcpp/src/service_server_01.cpp
touch example_service_rclcpp/src/service_client_01.cpp
```

面向对象方式写两个最简单的节点

`service_server_01.cpp`

```cpp
#include "rclcpp/rclcpp.hpp"

class ServiceServer01 : public rclcpp::Node {
 public:
  ServiceServer01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
  }

 private:
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServiceServer01>("service_server_01");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

`service_client_01.cpp`

```cpp
#include "rclcpp/rclcpp.hpp"

class ServiceClient01 : public rclcpp::Node {
 public:
  // 构造函数,有一个参数为节点名称
  ServiceClient01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
  }
 private:
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<ServiceClient01>("service_client_01");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

`CMakeLists.txt`

```cmake
add_executable(service_client_01 src/service_client_01.cpp)
ament_target_dependencies(service_client_01 rclcpp)

add_executable(service_server_01 src/service_server_01.cpp)
ament_target_dependencies(service_server_01 rclcpp)

install(TARGETS
  service_server_01
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  service_client_01
  DESTINATION lib/${PROJECT_NAME}
)
```

完成上面的步骤，即可编译测试了。

```bash
cd chapt3/chapt3_ws/
colcon build --packages-select example_service_rclcpp

# 运行 service_server_01
source install/setup.bash
ros2 run example_service_rclcpp service_server_01

# 打开新终端运行  service_client_01
source install/setup.bash
ros2 run example_service_rclcpp service_client_01
```

### 5.2 服务端实现

#### 5.2.1 导入接口

两数相加我们需要利用ROS2自带的`example_interfaces`接口，使用命令行可以查看这个接口的定义。

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

结果

```bash
int64 a
int64 b
---
int64 sum
```

导入接口的三个步骤：

> `ament_cmake`类型功能包导入消息接口分为三步：
>
> 1. 在`CMakeLists.txt`中导入，具体是先`find_packages`再`ament_target_dependencies`。
> 2. 在`packages.xml`中导入，具体是添加`depend`标签并将消息接口写入。
> 3. 在代码中导入，C++中是`#include"消息功能包/xxx/xxx.hpp"`。

根据步骤改一下：

`CMakeLists.txt`

```cmake
# 这里我们一次性把服务端和客户端对example_interfaces的依赖都加上
find_package(example_interfaces REQUIRED)

add_executable(service_client_01 src/service_client_01.cpp)
ament_target_dependencies(service_client_01 rclcpp example_interfaces)

add_executable(service_server_01 src/service_server_01.cpp)
ament_target_dependencies(service_server_01 rclcpp example_interfaces)
```

`packages.xml`

```xml
	<depend>example_interfaces</depend>
```

代码

```cpp
#include "example_interfaces/srv/add_two_ints.hpp"
```

#### 5.2.2 编写代码

```c++
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

class ServiceServer01 : public rclcpp::Node {
 public:
  ServiceServer01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    // 创建服务
    add_ints_server_ =
      this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints_srv",
        std::bind(&ServiceServer01::handle_add_two_ints, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

 private:
  // 声明一个服务
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr
    add_ints_server_;

  // 收到请求的处理函数
  void handle_add_two_ints(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a,
                request->b);
    response->sum = request->a + request->b;
  };
};
```

create_service，参考rclcpp API文档即可

![image-20220606222615670](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606222615670.png)

- ServiceT，消息接口`example_interfaces::srv::AddTwoInts`
- service_name，服务名称
- callback，回调函数，使用成员函数作为回调函数，std::bind进行转换
- qos_profile，服务质量配置文件，默认`rmw_qos_profile_services_default`
- group，调用服务的回调组，默认`nullptr`

#### 5.2.3 测试

```bash
cd chapt3_ws/
colcon build --packages-select example_service_rclcpp
source install/setup.bash
ros2 run example_service_rclcpp service_server_01
```

接着打开一个新的终端

```bash
# 你应该可以看到我们声明的服务
ros2 service list
# 使用命令行进行调用
ros2 service call /add_two_ints_srv example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"
```

![image-20220606224245543](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606224245543.png)

### 5.3 客户端实现

#### 5.3.1 API接口

写代码时看API文档是个好习惯，先看看创建客户端的：[地址](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html#aed42f345ae1de3a1979d5a8076127199)

##### 5.3.1.1 create_client

![image-20220606224628001](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606224628001.png)

参数加上ServiceT（接口类型），一共有四个，都是老熟人了，就不介绍了。

##### 5.3.1.2 async_send_request

接着我们来看看发送请求的API，[地址](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Client.html#a62e48edd618bcb73538bfdc3ee3d5e63)

![image-20220606224824684](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606224824684.png)

我们这里要用的是这个函数`async_send_request()`同时传入两个参数

- request，请求的消息，这里用于放a，b两个数。
- CallBack，回调函数，异步接收服务器的返回的函数。

> 至于为什么ROS2中那么多回调函数，以及用回调函数的好处，小鱼这里就不解释了，不清楚的小伙伴可以看看基础篇的内容。

##### 5.3.1.3 wait_for_service

这个函数是用于等待服务上线的，这个函数并不在rclcpp::Client中定义，而是在其[父类](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1ClientBase.html#a8b4b432338a460ceb26a7fa6ddd59e1d)中定义的。

![image-20220606225526647](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606225526647.png)

上面是继承图，在其父类中有这个函数的解释。

![image-20220606225411304](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606225411304.png)

参数就一个，等待的时间，返回值是bool类型的，上线了就是true，不上线就是false。

之所以会用的这个函数的原因是，再发送请求之前保证服务端启动了，避免发送一个请求出去而无人响应的尴尬局面。

#### 5.3.2 代码

```c++
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceClient01 : public rclcpp::Node {
 public:
  // 构造函数,有一个参数为节点名称
  ServiceClient01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    // 创建客户端
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_srv");
  }

  void send_request(int a, int b) {
    RCLCPP_INFO(this->get_logger(), "计算%d+%d", a, b);

    // 1.等待服务端上线
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      //等待时检测rclcpp的状态
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 2.构造请求的
    auto request =
      std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
    request->a = a;
    request->b = b;

    // 3.发送异步请求，然后等待返回，返回时调用回调函数
    client_->async_send_request(
      request, std::bind(&ServiceClient01::result_callback_, this,
                         std::placeholders::_1));
  };

 private:
  // 声明客户端
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

  void result_callback_(
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture
      result_future) {
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "计算结果：%ld", response->sum);
  }
};
```

回调函数`void result_callback_(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future)`

参数是客户端`AddTwoInts`类型的`SharedFuture`对象，这个对象的定义如下

![image-20220606230244527](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606230244527.png)

可以看到其又是利用C++11的新特性`std::shared_future`创建的`SharedResponse`类模板。

类模板 `std::shared_future` 提供访问异步操作结果的机制，类似 [std::future](https://www.apiref.com/cpp-zh/cpp/thread/future.html) ，除了允许多个线程等候同一共享状态。

具体看看[std::shared_future的API](https://www.apiref.com/cpp-zh/cpp/thread/shared_future.html)

![image-20220606231019702](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606231019702.png)

可以看到使用`get`函数即可获取结果。

```cpp
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "计算结果：%ld", response->sum);
```

#### 5.3.3 测试

最后还要修改下主函数，用于调用服务端发送请求。

```c++
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<ServiceClient01>("service_client_01");
  /* 运行节点，并检测退出信号*/
  //增加这一行，node->send_request(5, 6);，计算5+6结果
  node->send_request(5, 6);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

接着编译运行客户端

```bash
cd chapt3_ws/
colcon build --packages-select example_service_rclcpp
source install/setup.bash
ros2 run example_service_rclcpp service_client_01
```

打开服务端，让服务上线

```bash
source install/setup.bash
ros2 run example_service_rclcpp service_server_01
```

![image-20220606231650401](https://fishros.com/d2lros2/humble/chapt3/get_started/5.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLCPP%E5%AE%9E%E7%8E%B0/imgs/image-20220606231650401.png)

## 6 服务之RCLPY实现

### 6.1 创建功能包和节点

创建功能包其实还可以加上一些参数`--node-name 节点名`，让这个过程变得更简单。

```bash
cd chapt3/chapt3_ws/src
ros2 pkg create example_service_rclpy --build-type ament_python --dependencies rclpy example_interfaces  --node-name service_server_02
```

其中 `--node-name service_server_02`会帮你创建好节点文件和添加执行文件。但是也有一些限制，比如只支持一个节点文件，所以还需要手动创建一个。

```bash
cd example_service_rclpy/example_service_rclpy/
touch service_client_02.py
```

修改下setup.py

```python
    entry_points={
        'console_scripts': [
            "service_client_02 = example_service_rclpy.service_client_02:main",
            "service_server_02 = example_service_rclpy.service_server_02:main"
        ],
    },
```

接着面向对象来一筐，将两个节点的内容补充一下

`service_server_02`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ServiceServer02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        
def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ServiceServer02("service_server_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

```

`service_client_02`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ServiceClient02("service_client_02")  # 新建一个节点
    node.send_request(3,4)
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

### 6.2 服务端实现

#### 6.2.1 看 API

- [Node — rclpy 0.6.1 documentation (ros2.org)](https://docs.ros2.org/latest/api/rclpy/api/node.html)

![image-20220606233039489](https://fishros.com/d2lros2/humble/chapt3/get_started/6.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220606233039489.png)

#### 6.2.2 写代码

```python
# 导入接口
from example_interfaces.srv import AddTwoInts

class ServiceServer02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.add_ints_server_ = self.create_service(AddTwoInts,"add_two_ints_srv", self.handle_add_two_ints) 

    def handle_add_two_ints(self,request, response):
        self.get_logger().info(f"收到请求，计算{request.a} + {request.b}")
        response.sum = request.a + request.b
        return response
```

#### 6.2.3 测试

```bash
colcon build --packages-select example_service_rclpy
source install/setup.bash
ros2 run example_service_rclpy service_server_02
```

打开新终端

```bash
ros2 service call /add_two_ints_srv example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"
```

![image-20220606233619434](https://fishros.com/d2lros2/humble/chapt3/get_started/6.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220606233619434.png)

### 6.3 客户端实现

#### 6.3.1 API

- [Node — rclpy 0.6.1 documentation (ros2.org)](https://docs.ros2.org/latest/api/rclpy/api/node.html)

#### 6.3.2 写代码

```python
from example_interfaces.srv import AddTwoInts

class ServiceClient02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.client_ = self.create_client(AddTwoInts,"add_two_ints_srv") 

    def result_callback_(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"收到返回结果：{response.sum}")
    
    def send_request(self, a, b):
        while rclpy.ok() and self.client_.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
            
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.client_.call_async(request).add_done_callback(self.result_callback_)
        
def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ServiceClient02("service_client_02")  # 新建一个节点
    # 调用函数发送请求
    node.send_request(3,4)
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

同样是异步请求，rclpy客户端库定义的是`call_async`并且使用`add_done_callback`添加回调函数。

#### 6.3.3 测试

编译启动客户端

```bash
colcon build --packages-select example_service_rclpy
source install/setup.bash
ros2 run example_service_rclpy service_client_02
```

启动服务端

```bash
source install/setup.bash
ros2 run example_service_rclpy service_server_02
```

![image-20220606234546696](https://fishros.com/d2lros2/humble/chapt3/get_started/6.%E6%9C%8D%E5%8A%A1%E4%B9%8BRCLPY%E5%AE%9E%E7%8E%B0/imgs/image-20220606234546696.png)

## 7 ROS2接口介绍

### 7.1 接口

#### 7.1.1 什么是接口

**接口其实是一种规范**

此前使用过的两种接口分别代表字符串和32位二进制的整型数据，是ROS2提前定义的一种规范。

```bash
std_msgs/msg/String
std_msgs/msg/UInt32
```

#### 7.1.2 为什么使用接口

举一个雷达的例子，不同的厂家生产出不同的类型的激光雷达，每种雷达驱动方式、扫描速率等等都不相同。

当机器人进行导航时，需要激光雷达的扫描数据，假如没有统一接口，每次更换一个种类的雷达，都需要重新做程序适配。

于是ROS2中定义了一个统一的接口叫做`sensor_msgs/msg/LaserScan`，现在几乎每个雷达的厂家都会编写程序将自己雷达的数据变成`sensor_msgs/msg/LaserScan`格式，提供给用户使用。

#### 7.1.3 ROS2自带的接口

前面话题通信时`std_msgs`功能包是我们安装ROS2的时候ROS2为我们自动安装的，除了`std_msgs`之外，`ROS2`还定义了很多做机器人常用的接口。

> 使用`ros2 interface package sensor_msgs`命令可以查看某一个接口包下所有的接口

比如：传感器类的消息包`sensor_msgs`

```bash
打开终端输入：ros2 interface package sensor_msgs
sensor_msgs/msg/JointState  #机器人关节数据
sensor_msgs/msg/Temperature #温度数据
sensor_msgs/msg/Imu #加速度传感器
sensor_msgs/msg/Image #图像
sensor_msgs/msg/LaserScan #雷达数据
......
```

虽然ROS2为我们定义了大量`有手就行，拿来就用`的接口，但有时候还是不能满足实际需求，所以我们需要掌握自定义接口的方法。

### 7.2 接口文件内容

#### 7.2.1 可以定义的接口三种类型

ROS2提供了四种通信方式：

- 话题-Topics
- 服务-Services
- 动作-Action
- 参数-Parameters

除了参数之外，话题、服务和动作(Action)都支持自定义接口，每一种通信方式所适用的场景各不相同，所定义的接口也被分为话题接口、服务接口、动作接口三种。

#### 7.2.2 接口形式

这三种接口定义起来有什么不一样的地方呢？

话题接口格式：`xxx.msg`

```bash
int64 num
```

服务接口格式：`xxx.srv`

```bash
int64 a
int64 b
---
int64 sum
```

动作接口格式：`xxx.action`

```bash
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

#### 7.2.3 接口数据类型

根据引用方式不同可以分为基础类型和包装类型两类。

**基础类型**有（同时后面加上[]可形成数组）

```bash
bool
byte
char
float32,float64
int8,uint8
int16,uint16
int32,uint32
int64,uint64
string
```

**包装类型**

即在已有的接口类型上进行包含，比如

```bash
uint32 id
string image_name
sensor_msgs/Image
```

#### 7.2.4 接口如何生成代码

变量类型和名称定义好之后如何在程序里面调用呢？

其实这里有一个转换的过程：将msg、srv、action文件转换为Python和C++的头文件。

msg、srv、actionROS2-IDL转换器Python的py、C++的.h头文件

通过ROS2的IDL模块 产生了头文件，有了头文件，我们就可以在程序里导入并使用这个消息模块。

### 7.3 自定义接口实践

#### 7.3.1 场景定义

给定一个机器人开发中的常见控制场景，我们设计满足要求的服务接口和话题接口。

设计两个节点

- 一个机器人节点，对外提供移动指定距离服务，移动完成后返回当前位置，同时对外发布机器人的位置和状态（是否在移动）。
- 机器人控制节点，通过服务控制机器人移动指定距离，并实时获取机器人的当前位置和状态。

假设机器人在坐标轴上，只能前后移动。

#### 7.3.2 定义接口

服务接口`MoveRobot.srv`

```bash
# 前进后退的距离
float32 distance
---
# 当前的位置
float32 pose
```

话题接口，采用基础类型 `RobotStatus.msg`

```bash
uint32 STATUS_MOVEING = 1
uint32 STATUS_STOP = 1
uint32  status
float32 pose
```

话题接口，混合包装类型 `RobotPose.msg`

```bash
uint32 STATUS_MOVEING = 1
uint32 STATUS_STOP = 2
uint32  status
geometry_msgs/Pose pose
```

#### 7.3.3 创建接口功能包编接口

创建功能包

```bash
ros2 pkg create example_ros2_interfaces --build-type ament_cmake --dependencies rosidl_default_generators geometry_msgs
```

注意功能包类型必须为：ament_cmake

依赖`rosidl_default_generators`必须添加，`geometry_msgs`视内容情况添加（我们这里有`geometry_msgs/Pose pose`所以要添加）。

接着创建文件夹和文件将3.2中文件写入，注意话题接口放到`msg`文件夹下，以`.msg`结尾。服务接口放到`srv`文件夹下，以`srv`结尾。

```bash
.
├── CMakeLists.txt
├── msg
│   ├── RobotPose.msg
│   └── RobotStatus.msg
├── package.xml
└── srv
    └── MoveRobot.srv

2 directories, 5 files
```

接着修改`CMakeLists.txt`

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
# 添加下面的内容
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/RobotStatus.msg"
  "srv/MoveRobot.srv"
  DEPENDENCIES geometry_msgs
)
```

接着修改`package.xml`

```xml
  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rosidl_default_generators</depend>
  <depend>geometry_msgs</depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group> #添加这一行

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
```

保存即可编译

```bash
colcon build --packages-select example_ros2_interfaces
```

编译完成后在`chapt3_ws/install/example_ros2_interfaces/include`下应该可以看到C++的头文件。在`chapt3_ws/install/example_ros2_interfaces/local/lib/python3.10/dist-packages`下应该可以看到Python版本的头文件。

接下来的代码里我们就可以通过头文件导入和使用我们定义的接口了。

> 为什么要source？
>
> `source` 命令在脚本开发和系统管理中是一个非常有用的工具，特别是在需要设置或修改当前 shell 环境时。

### 7.4 ROS2接口常用CLI命令

#### 7.4.1 查看接口列表

```bash
ros2 interface list
```

![image-20210809161530132](https://fishros.com/d2lros2/humble/chapt3/get_started/7.ROS2%E6%8E%A5%E5%8F%A3%E4%BB%8B%E7%BB%8D/imgs/image-20210809161530132.png)

#### 7.4.2 查看某一个接口详细的内容

```bash
ros2 interface show std_msgs/msg/String
```

![image-20210809161933104](https://fishros.com/d2lros2/humble/chapt3/get_started/7.ROS2%E6%8E%A5%E5%8F%A3%E4%BB%8B%E7%BB%8D/imgs/image-20210809161933104.png)

## 8 自定义接口RCLCPP实战

本节我们就利用上一节创建好的消息接口进行代码编写代码，学习在实际的项目中使用自定义接口，同时也作为一个小练习，我们将在同一个节点里融合话题和服务两种通信方式。

### 8.1 创建功能包和节点

这里我们设计两个节点

- `example_interfaces_robot_01`，机器人节点，对外提供控制机器人移动服务并发布机器人的状态。
- `example_interfaces_control_01`，控制节点，发送机器人移动请求，订阅机器人状态话题。

在工作空间下的src文件夹中创建功能包`example_ros2_interfaces`添加`example_ros2_interfaces`和`rclcpp`依赖，并自动生成`example_interfaces_robot_01`节点。

因为`--node-name`只支持创建一个节点，我们再添加一下另外一个节点。

```bash
cd chapt3_ws/
ros2 pkg create example_interfaces_rclcpp --build-type ament_cmake --dependencies rclcpp example_ros2_interfaces --destination-directory src --node-name example_interfaces_robot_01 
touch src/example_interfaces_rclcpp/src/example_interfaces_control_01.
```

`CMakeLists.txt`

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_ros2_interfaces REQUIRED)

add_executable(example_interfaces_robot_01 src/example_interfaces_robot_01.cpp)
target_include_directories(example_interfaces_robot_01 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_interfaces_robot_01 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  example_interfaces_robot_01
  "rclcpp"
  "example_ros2_interfaces"
)

install(TARGETS example_interfaces_robot_01
  DESTINATION lib/${PROJECT_NAME})

add_executable(example_interfaces_control_01 src/example_interfaces_control_01.cpp)
target_include_directories(example_interfaces_control_01 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_interfaces_control_01 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  example_interfaces_control_01
  "rclcpp"
  "example_ros2_interfaces"
)

install(TARGETS example_interfaces_control_01
  DESTINATION lib/${PROJECT_NAME})
```

面向对象写一下两个节点的内容

`example_interfaces_control_01.cpp`

```c++
 #include "rclcpp/rclcpp.hpp"

class ExampleInterfacesControl : public rclcpp::Node {
 public:
  // 构造函数,有一个参数为节点名称
  ExampleInterfacesControl(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
  }

 private:
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleInterfacesControl>("example_interfaces_control_01");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

`example_interfaces_robot_01.cpp`

```c++
#include "rclcpp/rclcpp.hpp"

/*创建一个机器人类，模拟真实机器人*/
class Robot {
 public:
  Robot() = default;
  ~Robot() = default;
 private:
};


class ExampleInterfacesRobot : public rclcpp::Node {
 public:
  ExampleInterfacesRobot(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
  }

 private:
  Robot robot;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleInterfacesRobot>("example_interfaces_robot_01");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

保存编译即可测试

```bash
cd chapt3_ws/
colcon build
source install/setup.bash
```

### 8.2 编写机器人类

```c++
// 导入上一节定义的消息接口
#include "example_ros2_interfaces/msg/robot_status.hpp"
#include "example_ros2_interfaces/srv/move_robot.hpp"
#include "rclcpp/rclcpp.hpp"

/*
 * 测试指令：ros2 service call /move_robot example_ros2_interfaces/srv/MoveRobot "{distance: 5}"
 */

class Robot {
 public:
  Robot() = default;
  ~Robot() = default;
  /**
   * @brief 移动指定的距离
   *
   * @param distance
   * @return float
   */
  float move_distance(float distance) {
    status_ = example_ros2_interfaces::msg::RobotStatus::STATUS_MOVEING;
    target_pose_ += distance;
    // 当目标距离和当前距离大于0.01则持续向目标移动
    while (fabs(target_pose_ - current_pose_) > 0.01) {
      // 每一步移动当前到目标距离的1/10
      float step = distance / fabs(distance) * fabs(target_pose_ - current_pose_) * 0.1;
      current_pose_ += step;
      std::cout << "移动了：" << step << "当前位置：" << current_pose_ << std::endl;
      // 当前线程休眠500ms
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    status_ = example_ros2_interfaces::msg::RobotStatus::STATUS_STOP;
    return current_pose_;
  }
  /**
   * @brief Get the current pose
   *
   * @return float
   */
  float get_current_pose() { return current_pose_; }

  /**
   * @brief Get the status
   *
   * @return int
   *  1 example_ros2_interfaces::msg::RobotStatus::STATUS_MOVEING
   *  2 example_ros2_interfaces::msg::RobotStatus::STATUS_STOP
   */
  int get_status() { return status_; }

 private:
  // 声明当前位置
  float current_pose_ = 0.0;
  // 目标距离
  float target_pose_ = 0.0;
  int status_ = example_ros2_interfaces::msg::RobotStatus::STATUS_STOP;
};
```

该类的实现比较简单，对外提供获取当前状态、获取当前位置和移动一定的距离三个接口，其中移动指定距离这个函数每移动一步会休眠500ms。

### 8.3 编写机器人节点逻辑

接着我们就可以利用接口编写机器人节点了

```c++
class ExampleInterfacesRobot : public rclcpp::Node {
 public:
  ExampleInterfacesRobot(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    /*创建move_robot服务*/
    move_robot_server_ = this->create_service<example_ros2_interfaces::srv::MoveRobot>(
      "move_robot", std::bind(&ExampleInterfacesRobot::handle_move_robot, this, std::placeholders::_1, std::placeholders::_2));
    /*创建发布者*/
    robot_status_publisher_ = this->create_publisher<example_ros2_interfaces::msg::RobotStatus>("robot_status", 10);
    /*创建一个周期为500ms的定时器*/
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ExampleInterfacesRobot::timer_callback, this));
  }

 private:
  Robot robot; /*实例化机器人*/
  rclcpp::TimerBase::SharedPtr timer_; /*定时器，用于定时发布机器人位置*/
  rclcpp::Service<example_ros2_interfaces::srv::MoveRobot>::SharedPtr move_robot_server_; /*移动机器人服务*/
  rclcpp::Publisher<example_ros2_interfaces::msg::RobotStatus>::SharedPtr robot_status_publisher_; /*发布机器人位姿发布者*/

  /**
   * @brief 500ms 定时回调函数，
   * 
   */
  void timer_callback() {
    // 创建消息
    example_ros2_interfaces::msg::RobotStatus message;
    message.status = robot.get_status();
    message.pose = robot.get_current_pose();
    RCLCPP_INFO(this->get_logger(), "Publishing: %f", robot.get_current_pose());
    // 发布消息
    robot_status_publisher_->publish(message);
  };

  /**
   * @brief 收到话题数据的回调函数
   * 
   * @param request 请求共享指针，包含移动距离
   * @param response 响应的共享指针，包含当前位置信息
   */
  void handle_move_robot(const std::shared_ptr<example_ros2_interfaces::srv::MoveRobot::Request> request,
                         std::shared_ptr<example_ros2_interfaces::srv::MoveRobot::Response> response) {
    RCLCPP_INFO(this->get_logger(), "收到请求移动距离：%f，当前位置：%f", request->distance, robot.get_current_pose());
    robot.move_distance(request->distance);
    response->pose = robot.get_current_pose();
  };
};
```

逻辑也比较简单，利用定时器不断发送数据，收到请求后调用机器人类的move_distance接口来移动机器人。

### 8.4 编写控制节点

控制节点类代码

头文件部分

```C++
#include "rclcpp/rclcpp.hpp"
#include "example_ros2_interfaces/srv/move_robot.hpp"
#include "example_ros2_interfaces/msg/robot_status.hpp" 
```

`ExampleInterfacesControl`类

```c++
class ExampleInterfacesControl : public rclcpp::Node {
 public:
  ExampleInterfacesControl(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    /*创建move_robot客户端*/
    client_ = this->create_client<example_ros2_interfaces::srv::MoveRobot>(
      "move_robot");
    /*订阅机器人状态话题*/
    robot_status_subscribe_ = this->create_subscription<example_ros2_interfaces::msg::RobotStatus>("robot_status", 10, std::bind(&ExampleInterfacesControl::robot_status_callback_, this, std::placeholders::_1));
  }


  /**
   * @brief 发送移动机器人请求函数
   * 步骤：1.等待服务上线
   *      2.构造发送请求
   * 
   * @param distance 
   */
  void move_robot(float distance) {
    RCLCPP_INFO(this->get_logger(), "请求让机器人移动%f", distance);

    /*等待服务端上线*/
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      //等待时检测rclcpp的状态
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 构造请求
    auto request = 
      std::make_shared<example_ros2_interfaces::srv::MoveRobot::Request>();
    request->distance = distance;

    // 发送异步请求，然后等待返回，返回时调用回调函数
    client_->async_send_request(
      request, std::bind(&ExampleInterfacesControl::result_callback_, this,
                         std::placeholders::_1));
  };

 private:
  // 声明客户端
  rclcpp::Client<example_ros2_interfaces::srv::MoveRobot>::SharedPtr client_;
  rclcpp::Subscription<example_ros2_interfaces::msg::RobotStatus>::SharedPtr robot_status_subscribe_;
  /* 机器人移动结果回调函数 */
  void result_callback_(
    rclcpp::Client<example_ros2_interfaces::srv::MoveRobot>::SharedFuture
      result_future) {
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "收到移动结果：%f", response->pose);
  }

  /**
   * @brief 机器人状态话题接收回调函数
   * 
   * @param msg 
   */
  void robot_status_callback_(const example_ros2_interfaces::msg::RobotStatus::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到状态数据位置：%f 状态：%d", msg->pose ,msg->status);
  }
};
```

main函数

```c++
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleInterfacesControl>("example_interfaces_control_01");
  /*这里调用了服务，让机器人向前移动5m*/
  node->move_robot(5.0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 8.5 测试运行

#### 8.5.1 编译

```bash
colcon build --packages-up-to example_interfaces_rclcpp
```

又遇到了个新的指令`--packages-up-to`：编译一个节点及其依赖。使用这个指令会以先后顺序先编译`example_ros2_interfaces`再编译`example_interfaces_rclcpp`。

![image-20220608210644665](https://fishros.com/d2lros2/humble/chapt3/get_started/8.%E8%87%AA%E5%AE%9A%E4%B9%89%E6%8E%A5%E5%8F%A3RCLCPP%E5%AE%9E%E6%88%98/imgs/image-20220608210644665.png)

#### 8.5.2 测试

`控制端`

```bash
source install/setup.bash
ros2 run example_interfaces_rclcpp example_interfaces_control_01
```

`服务端`

```bash
source install/setup.bash
ros2 run example_interfaces_rclcpp  example_interfaces_robot_01
```

服务端启动后机器人开始移动，时间为：`1654693733.053691559`

![image-20220608210907257](https://fishros.com/d2lros2/humble/chapt3/get_started/8.%E8%87%AA%E5%AE%9A%E4%B9%89%E6%8E%A5%E5%8F%A3RCLCPP%E5%AE%9E%E6%88%98/imgs/image-20220608210907257.png)

移动结束，`收到移动结果：4.990017`，时间为：`1654693763.501926752`

![image-20220608210939979](https://fishros.com/d2lros2/humble/chapt3/get_started/8.%E8%87%AA%E5%AE%9A%E4%B9%89%E6%8E%A5%E5%8F%A3RCLCPP%E5%AE%9E%E6%88%98/imgs/image-20220608210939979.png)

#### 8.5.3 思考

虽然机器人可以移动，客户端也可以收到机器人的位置，但是在机器人移动期间，控制端就收不到来自机器人端的实时位置信息的话题发布。

原因是服务端调用机器人移动的时候造成了主线程的阻塞和休眠，只有机器人完成移动后才会退出，造成了发布数据的定时器回调无法正常进行。

解决这个问题的方法有很多，比如开个单独给服务开个线程，比如换一种通信方式。

## 9 自定义接口RCLPY实战

以RCLPY客户端库为例，使用RCLPY的API通过自定义接口实现控制节点和机器人节点之间的话题与服务通信。

### 9.1 创建功能包

这里依然设计两个节点

- `example_interfaces_robot_02`，机器人节点，对外提供控制机器人移动服务并发布机器人的状态。
- `example_interfaces_control_02`，控制节点，发送机器人移动请求，订阅机器人状态话题。

```bash
cd chapt3_ws/src
ros2 pkg create example_interfaces_rclpy --build-type ament_python --dependencies rclpy example_ros2_interfaces --node-name example_interfaces_robot_02
touch example_interfaces_rclpy/example_interfaces_rclpy/example_interfaces_control_02.py
```

`setup.py`

```python
entry_points={
        'console_scripts': [
            'example_interfaces_control_02 = example_interfaces_rclpy.example_interfaces_control_02:main',
            'example_interfaces_robot_02 = example_interfaces_rclpy.example_interfaces_robot_02:main'
        ],
    },
```

`example_interfaces_robot_02.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Robot():
    def __init__(self) -> None:
        pass        

class ExampleInterfacesRobot02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        
def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ExampleInterfacesRobot02("example_interfaces_robot_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

`example_interfaces_control_02.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ExampleInterfacesControl02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ExampleInterfacesControl02("example_interfaces_control_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

编译测试

```bash
# 新终端
colcon build --packages-up-to example_interfaces_rclpy
source install/setup.bash
ros2 run example_interfaces_rclpy example_interfaces_robot_02
# 新终端
source install/setup.bash
ros2 run example_interfaces_rclpy example_interfaces_control_02
```

![image-20220612102321258](https://fishros.com/d2lros2/humble/chapt3/get_started/9.%E8%87%AA%E5%AE%9A%E4%B9%89%E6%8E%A5%E5%8F%A3RCLPY%E5%AE%9E%E6%88%98/imgs/image-20220612102321258.png)

### 9.2 编写机器人类

源码与解析

```python
from example_ros2_interfaces.msg import RobotStatus
import math
from time import sleep

class Robot():
    def __init__(self) -> None:
        self.current_pose_ = 0.0
        self.target_pose_ = 0.0
        self.status_ = RobotStatus.STATUS_STOP

    def get_status(self):
        return self.status_

    def get_current_pose(self):
        return self.current_pose_

    def move_distance(self,distance):
        self.status_ = RobotStatus.STATUS_MOVEING # 更新状态为移动、
        self.target_pose_ += distance # 更新目标位置

        while math.fabs(self.target_pose_ - self.current_pose_) > 0.01:
            step = distance / math.fabs(distance) * math.fabs(self.target_pose_ - self.current_pose_) * 0.1 # 计算一步移动距离
            self.current_pose_  += step # 移动一步
            print(f"移动了：{step}当前位置：{self.current_pose_}")
            sleep(0.5) #休息0.5s
        self.status_ = RobotStatus.STATUS_STOP # 更新状态为停止
        return self.current_pose_
```

### 9.3 编写机器人节点

```python
from example_ros2_interfaces.srv import MoveRobot

class ExampleInterfacesRobot02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.robot = Robot()
        self.move_robot_server_ = self.create_service(MoveRobot,"move_robot", self.handle_move_robot) 
        self.robot_status_publisher_ = self.create_publisher(RobotStatus,"robot_status", 10) 
        self.publisher_timer_ = self.create_timer(0.5, self.publisher_timer_callback)

    def publisher_timer_callback(self):
        """
        定时器回调发布数据函数
        """
        msg = RobotStatus() #构造消息
        msg.status = self.robot.get_status()
        msg.pose = self.robot.get_current_pose()
        self.robot_status_publisher_.publish(msg) # 发布消息
        self.get_logger().info(f'发布了当前的状态：{msg.status} 位置：{msg.pose}')

    def handle_move_robot(self,request, response):
        self.robot.move_distance(request.distance)
        response.pose = self.robot.get_current_pose()
        return response
```

逻辑与RCLCPP版本一致，创建服务和发布者，并创建定时器定时调用发布者完成发布。

### 9.4 编写控制节点

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_ros2_interfaces.msg import RobotStatus
from example_ros2_interfaces.srv import MoveRobot

class ExampleInterfacesControl02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.client_ = self.create_client(MoveRobot,"move_robot") 
        self.robot_status_subscribe_ = self.create_subscription(RobotStatus,"robot_status",self.robot_status_callback,10)

    def robot_status_callback(self,msg):
        self.get_logger().info(f"收到状态数据位置：{msg.pose} 状态：{msg.status}")

    def move_result_callback_(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"收到返回结果：{response.pose}")

    def move_robot(self, distance):
        while rclpy.ok() and self.client_.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
        request = MoveRobot.Request()
        request.distance = distance
        self.get_logger().info(f"请求服务让机器人移动{distance}")
        self.client_.call_async(request).add_done_callback(self.move_result_callback_)


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ExampleInterfacesControl02("example_interfaces_control_02")  # 新建一个节点
    node.move_robot(5.0) #移动5米
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

控制节点逻辑也与`RCLCPP`版本一致，创建一个订阅者和客户端，在主函数中请求服务端进行移动。

### 9.5 运行测试

```bash
# 新终端
colcon build --packages-up-to example_interfaces_rclpy
source install/setup.bash
ros2 run example_interfaces_rclpy example_interfaces_robot_02
# 新终端
source install/setup.bash
ros2 run example_interfaces_rclpy example_interfaces_control_02
```

同样的，在机器人移动期间机器人节点并没有发布机器人的位姿出来，在进阶篇中我们可以使用ROS2的多线程执行器和回调组来解决这个问题。

![image-20220612105944206](https://fishros.com/d2lros2/humble/chapt3/get_started/9.%E8%87%AA%E5%AE%9A%E4%B9%89%E6%8E%A5%E5%8F%A3RCLPY%E5%AE%9E%E6%88%98/imgs/image-20220612105944206.png)

![image-20220612110007991](https://fishros.com/d2lros2/humble/chapt3/get_started/9.%E8%87%AA%E5%AE%9A%E4%B9%89%E6%8E%A5%E5%8F%A3RCLPY%E5%AE%9E%E6%88%98/imgs/image-20220612110007991.png)



# 进阶篇——中间件进阶

## 1 原始数据类型与包装类型

在ROS2中定义接口，需要编写一个接口文件，该文件后缀为`msg`、`srv`、`action`。

在接口文件中定义通信过程中所使用的数据类型和数据名称，那可用的数据类型和数据名称有哪些呢？

### 1.1 数据名称

数据名称就是一个字符串，没啥好说的，符合编程语言变量的命名规则就行（比如不能是数字开头）

### 1.2 数据类型

#### 1.2.1 数据类型有哪些呢？

原始的数据类型只有九类。其中每一个都可以在后面加上`[]`将其变成数组形式（从一个变成多个）

```bash
bool
byte
char
float32, float64
int8, uint8
int16, uint16
int32, uint32
int64, uint64
string
```

> 上面这九类中，官方也在考虑新增一些和删除一些，目前还是支持的。

#### 1.2.2 类型扩展（套娃）

##### 1.2.2.1 第一层套娃

ROS2基于上面的九类基础数据类型，为我们定义出了很多拿来就用的数据类型，比如我们在前面章节中用到的图像数据类型`sensor_msgs/Image`，我们可以使用下面的命令来看一下其组成：

```bash
ros2 interface show sensor_msgs/msg/Image
```

![image-20210824191624340](https://fishros.com/d2lros2/humble/chapt3/advanced/1.%E5%8E%9F%E5%A7%8B%E6%95%B0%E6%8D%AE%E7%B1%BB%E5%9E%8B%E4%B8%8E%E5%8C%85%E8%A3%85%E7%B1%BB%E5%9E%8B/imgs/image-20210824191624340.png)

去掉单行的注释后的样子如下：

```bash
std_msgs/Header header # Header timestamp should be acquisition time of image
uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns
string encoding       # Encoding of pixels -- channel meaning, ordering, size
uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
```

我们可以看到，除了第一行`std_msgs/Header header`之外的其他部分都是由基础类型组成。

##### 1.2.2.2 第二层套娃

那`std_msgs/Header`由什么组成呢？我们再次使用下面的指令查看一下：

```bash
ros2 interface show std_msgs/msg/Header
```

结果如下：

```bash
builtin_interfaces/Time stamp # Two-integer timestamp that is expressed as seconds and nanoseconds.
string frame_id # Transform frame with which this data is associated.
```

##### 1.2.2.3 第三层套娃

看完上面的结果，除了基本类型`string`和我们发现还有一层`builtin_interfaces/Time`，我们再查看一下这个接口类型。

```bash
ros2 interface show builtin_interfaces/msg/Time 
```

结果如下：

```bash
# Time indicates a specific point in time, relative to a clock's 0 point.

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 10e9).
uint32 nanosec
```

我们发现结果全都是基本类型了，终于把套娃给解开了。

### 1.3 接口类型总结

通过基本类型的组合，可以构成一个新的数据类型，而新的数据类型又可以和基本类型或者另外一个数据类型互相组成另一个数据类型。所以我们可以说ROS2中的数据类型有无数种。

## 2 [通信质量Qos配置指南](https://mp.weixin.qq.com/s/J63fO4c_QIseLGQd5W2fAw)

## 3 DDS进阶之Fast-DDS环境搭建

### 3.1 论FastDDS的三种打开方式

**FastDDS和普通ROS包一样，有二进制安装、源码编译、Docker三种安装方式。**这里选择从源码开始安装，并用colcon编译。

### 3.2 源码编译安装FastDDS

#### 3.2.1 安装工具和依赖库

安装工具

```bash
sudo apt install python3-colcon-common-extensions python3-vcstool zip openjdk-8-jdk  -y
```

安装依赖库

```bash
sudo apt-get install libasio-dev -y
```

#### 3.2.2 创建目录，下载仓库

```bash
mkdir -p fastdds_ws/src 
cd fastdds_ws
wget http://fishros.com/tools/files/fastrtps.repos
# 如果下载失败则手动创建 fastrtps.repos 文件，文件内容见注释
vcs import src < fastrtps.repos
```

> [安装Fast DDS依赖项的 repos 文件时出现404：Not Found](https://fishros.org.cn/forum/topic/79/安装fast-dds依赖项的-repos-文件时出现404-not-found/3?_=1650535091374)
>
> **无法下的文件内容如下，请直接创建**`fastrtps.repos`**文件并将下面的内容手动复制进去即可。**
> **完成后再调用**`vcs import src < fastrtps.repos`**即可**
>
> ```
> repositories:
>     foonathan_memory_vendor:
>         type: git
>         url: https://github.com/eProsima/foonathan_memory_vendor.git
>         version: master
>     fastcdr:
>         type: git
>         url: https://github.com/eProsima/Fast-CDR.git
>         version: master
>     fastrtps:
>         type: git
>         url: https://github.com/eProsima/Fast-DDS.git
>         version: master
>     fastddsgen:
>         type: git
>         url: https://github.com/eProsima/Fast-DDS-Gen.git
>         version: master
>     fastddsgen/thirdparty/idl-parser:
>         type: git
>         url: https://github.com/eProsima/IDL-Parser.git
>         version: master
> ```

#### 3.2.3 安装gradle 6.4

首先，从 Gradle 官方网站下载 Gradle 6.4 版本的二进制文件：

```sh
wget https://services.gradle.org/distributions/gradle-6.4-bin.zip
```

将下载的文件解压到 `/opt/gradle` 目录：

```sh
sudo unzip -d /opt/gradle gradle-6.4-bin.zip
```

为 Gradle 创建一个环境变量配置文件：

```sh
sudo nano /etc/profile.d/gradle.sh
```

在文件中添加以下内容：

```sh
export GRADLE_HOME=/opt/gradle/gradle-6.4
export PATH=${GRADLE_HOME}/bin:${PATH}
```

保存并关闭文件。

加载新的环境变量配置：

```sh
source /etc/profile.d/gradle.sh
```

最后，验证 Gradle 是否安装成功：

```sh
gradle -v
```

你应该会看到类似以下的输出，显示 Gradle 6.4 版本的信息：

```sh
------------------------------------------------------------
Gradle 6.4
------------------------------------------------------------

Build time:   2020-05-15 19:31:35 UTC
Revision:     1b0fa997b557e6c9ae8031196e60a76464d4b3a2

Kotlin:       1.3.72
Groovy:       2.5.10
Ant:          Apache Ant(TM) version 1.10.7 compiled on September 1 2019
JVM:          11.0.23 (Ubuntu 11.0.23+8-0ubuntu1~22.04)
OS:           Linux 5.15.0-25-generic amd64
```

> 8.0以上的高版本gradle不支持FastDDS。

#### 3.2.4 编译

```bash
colcon build
cd src/fastddsgen
gradle assemble
```

#### 3.2.5 最后一步:配置环境变量

xxx是你的目录前缀

```bash
echo 'source xxx/fastdds_ws/install/setup.bash' >> ~/.bashrc
# 采用3.2.3节中所属方法安装的gradle可以不必配置下面这一条环境变量
echo 'export PATH=$PATH:xxx/fastdds_ws/gradle-6.4/bin/' >> ~/.bashrc
echo 'export DDSGEN=xxx/fastdds_ws/src/fastddsgen/scripts' >> ~/.bashrc
```

## 4 使用DDS进行订阅发布

### 4.1 HelloFish例程

DDS使用的RTPS，就是Real-Time Publish Subscribe协议，其实和ROS与ROS2中的发布订阅是一样的，所以我们就跑一个例程来收发消息，消息内容就叫`HelloFish`

#### 4.1.1 下载代码

```bash
git clone https://github.com/fishros/dds_tutorial.git
```

#### 4.1.2 编译例程

```bash
cd dds_tutorial/examples/01-hellofishros
mkdir build && cd build
cmake .. 
make
```

#### 4.1.3 执行例程

开一个终端

```bash
./DDSHelloFishRosPublisher  
```

再开一个终端

```bash
./DDSHelloFishRosSubscribe
```

#### 4.1.4 查看结果

正确结果像下面这样子，已经证明一切OK了~ ![DDS发布订阅测试](https://fishros.com/d2lros2/humble/chapt3/advanced/4.%E4%BD%BF%E7%94%A8DDS%E8%BF%9B%E8%A1%8C%E8%AE%A2%E9%98%85%E5%8F%91%E5%B8%83/imgs/69d6079ecd16442cb3c6824b742ae705.png)

> 看到熟悉的发布订阅是不是很神奇，FASTDDS底层采用了多种协议进行数据的传输，包括不靠谱但真的很快的UDP，靠谱但是不怎么快的TCP，还有感觉不传输的内存交换（SHM)。