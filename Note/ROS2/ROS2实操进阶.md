# ROS2实操进阶

- [ROS2实操进阶](#ros2实操进阶)
  - [1 OOP介绍](#1-oop介绍)
    - [1.1 Why](#11-why)
    - [1.2 思想辨析](#12-思想辨析)
      - [1.2.1 用三种思想把大象装进冰箱](#121-用三种思想把大象装进冰箱)
        - [1.2.1.1 面向过程思想](#1211-面向过程思想)
        - [1.2.1.2 面向对象思想](#1212-面向对象思想)
        - [1.2.1.3 函数式思想](#1213-函数式思想)
    - [1.3 面向对象编程](#13-面向对象编程)
      - [1.3.1 类与对象（抽象与具体）](#131-类与对象抽象与具体)
      - [1.3.2 封装、继承与多态](#132-封装继承与多态)
    - [1.4 如何选择code思想](#14-如何选择code思想)
  - [2 使用面向对象方式编写ROS2节点](#2-使用面向对象方式编写ros2节点)
    - [2.1 C++版本](#21-c版本)
    - [2.2 Python版本](#22-python版本)
  - [3 Colcon使用进阶](#3-colcon使用进阶)
    - [3.1 ROS生态中的构建系统和构建工具](#31-ros生态中的构建系统和构建工具)
      - [3.1.1 构建系统与构建工具](#311-构建系统与构建工具)
      - [3.1.2 常见构建系统](#312-常见构建系统)
        - [3.1.2.1 CMake](#3121-cmake)
        - [3.1.2.2 Python setuptools](#3122-python-setuptools)
        - [3.1.2.3 catkin](#3123-catkin)
      - [3.1.3 常见构建工具](#313-常见构建工具)
        - [3.1.3.1 catkin\_make](#3131-catkin_make)
        - [3.1.3.2 colcon](#3132-colcon)
        - [3.1.3.3 ament\_tools](#3133-ament_tools)
    - [3.2.Colcon构建进阶](#32colcon构建进阶)
      - [3.2.1 build参数](#321-build参数)
        - [3.2.1.1 构建指令](#3211-构建指令)
        - [3.2.1.2 指定构建后安装的目录](#3212-指定构建后安装的目录)
        - [3.2.1.3 合并构建目录](#3213-合并构建目录)
        - [3.2.1.4 符号链接安装](#3214-符号链接安装)
        - [3.2.1.5 错误时继续安装](#3215-错误时继续安装)
        - [3.2.1.6 CMake参数](#3216-cmake参数)
        - [3.2.1.7 控制构建线程](#3217-控制构建线程)
        - [3.2.1.8 开启构建日志](#3218-开启构建日志)
  - [4 ROS2节点发现与多机通信](#4-ros2节点发现与多机通信)
    - [4.1 概述](#41-概述)
    - [4.2 选择域ID (短版本)](#42-选择域id-短版本)
    - [4.3 选择域ID (长版本)](#43-选择域id-长版本)
    - [4.4 特定平台的约束](#44-特定平台的约束)
    - [4.5 参与者约束](#45-参与者约束)
    - [4.6 域ID到UDP端口号计算器](#46-域id到udp端口号计算器)


## 1 OOP介绍

要做机器人离不开计算机编程，而计算机编程经过多年的发展，演变出了三种不同且常用的编程思想，分别是：

1. 面向过程编程思想。缩写：POP
2. 面向对象编程思想。缩写：OOP
3. 函数式思想。缩写：FP

### 1.1 Why

为什么了解这些编程思想呢？尤其是OOP。

如果用户对开发者的编程思想不了解，那么阅读机器人相关开源程序代码时，比如导航框架Nav2、机械臂运动控制框架Moveit，就会发现每一行好像都看得懂，但放一起就看不懂了，看函数调来调去，很快就蒙了，不知道如何下手。

通过了解常见的三种编程思想，脑子里有个概念，以后再遇到看不明白的程序，就知道该往哪个方向去学习。

### 1.2 思想辨析

首先明确一件事情，编程是为了什么？

是为了赚钱吗？

不，写程序肯定是为了解决实际的问题，那编程思想就是解决问题的思路（赚钱工具）

那这三种思想有什么区别呢？

#### 1.2.1 用三种思想把大象装进冰箱

比如我们想把一只大象装进冰箱，分别用三种思想，我们看看有什么不一样。

![image-20210918165813562](https://fishros.com/d2lros2/humble/chapt2/advanced/1.%E9%9D%A2%E5%90%91%E5%AF%B9%E8%B1%A1%E7%BC%96%E7%A8%8B%E6%80%9D%E6%83%B3/imgs/image-20210918165813562-16542707860001.png)

##### 1.2.1.1 面向过程思想

如果我们采用面向过程的思想，可以分为三步：

1. 打开冰箱门
2. 把大象塞进去
3. 关上冰箱门

面向过程编程就是分析出解决问题所需要步骤，然后分别实现每一步，再一步步执行即可。

##### 1.2.1.2 面向对象思想

面向对象编程思想（OOP）怎么做呢？

任何我们想要探究的事物都可以当作一个对象，比如我们可以把冰箱理解为一个对象，进而研究冰箱由哪些部分（指令装置等）组成，能干什么（制冷、调温等）？

接着我们开始下定义，

| 冰箱           | 定义       | 举例                     |
| -------------- | ---------- | ------------------------ |
| 冰箱的组成部分 | 冰箱的属性 | 制冷器，调温旋钮、灯带等 |
| 冰箱能干什么   | 冰箱的行为 | 制冷，调温、照明等       |

对象的行为其实是对其属性的操作，比如对制冷器操作就可以制冷，给灯带通电就可以照明。

**对象 = 属性+行为**

接着我们开始采用OOP的方法把大象装进冰箱

1. 调用：冰箱->打开门(行为)
2. 调用：冰箱->装东西(行为)
3. 调用：冰箱->关闭门(行为)

看起来和面向过程没啥区别，但我们的思想发生了重大的转变，我们把冰箱当作了一个独立的对象，我们是通过和冰箱这个对象交互完成了整个过程。

##### 1.2.1.3 函数式思想

接着来看函数式编程

1. 定义关进（冰箱，大象）函数
2. 实现函数：关门(放入(开门(冰箱)，大象))

可以看到多层的函数嵌套调用，这就是函数编程的魅力，但是FP不是ROS学习中的重点！

### 1.3 面向对象编程

#### 1.3.1 类与对象（抽象与具体）

我们通过调用你家美的冰箱的开门、装东西和关门三个行为来把大象装进冰箱。
这时我们可以把`你家的美的冰箱`（具体的）称之为一个对象，而冰箱（抽象的）就称为一个类。

比如说鱼类和小鱼，鱼类就是一个类，而小鱼就是鱼类（抽象的）中的一个对象（具体的）。

在ROS2设计时这种抽象和具体的思想发挥着非常重要的作用，比如说DDS是有很多厂家的，ROS2为了匹配不同厂家的DDS，就设计除了DDS抽象层，而每一个具体的DDS厂家，我们可以称之为一个DDS的对象，是具体的。

#### 1.3.2 封装、继承与多态

所谓**封装**就是将属性和行为封装在一起。上面已经介绍了**对象 = 属性+行为**，比如冰箱将冰箱的温度值（属性）和对温度值的操作（行为）等封装在一起。

**继承**，继承可以帮我们减少很多的工作量，比如ROS2中的执行器类，通过继承执行器类实现了单线程执行器和多线程执行器。

**多态**，其实很简单，我们可以说鲤鱼是鱼类，草鱼是鱼类，鲤鱼是鱼类。同一个鱼类可以有多种不同的类型，即多态。

### 1.4 如何选择code思想

三种编程思想，我们写程序的时候该如何选择呢?

根据实际的功能需求来，如果只需要实现一个很简单的功能，比如只是做一个键盘控制器，实现控制小车前进后退，直接采用面向过程的设计思想即可。

但如果是做一个稍大的工程，且后续要考虑功能的拓展性，这个时候就需要采用面向对象的思路来了。

参考链接：-[浅谈面向对象的编程思想：如何优雅地把大象装进冰箱？_SYSU_101的博客-CSDN博客](https://blog.csdn.net/SYSU_101/article/details/78057008)

## 2 使用面向对象方式编写ROS2节点

### 2.1 C++版本

在`Code/ROS2/Test/Cpp/example_ws/src/example_cpp/src`下新建`node_03.cpp`，接着输入下面的代码。

```c++
#include "rclcpp/rclcpp.hpp"

/*
    创建一个类节点，名字叫做Node03,继承自Node.
*/
class Node03 : public rclcpp::Node
{

public:
    // 构造函数,有一个参数为节点名称
    Node03(std::string name) : Node(name)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.",name.c_str());
    }

private:
   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个node_03的节点*/
    auto node = std::make_shared<Node03>("node_03");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

接着修改`CMakeLists.txt`，添加下方代码。

```cmake
add_executable(node_03 src/node_03.cpp)
ament_target_dependencies(node_03 rclcpp)

install(TARGETS
  node_03
  DESTINATION lib/${PROJECT_NAME}
)
```

接着即可自行编译测试

```bash
colcon build --packages-select example_cpp	# 实测可以直接 colcon build，无需参数
source install/setup.bash
ros2 run example_cpp node_03
```

![](https://s2.loli.net/2024/07/12/woa7hr6LQTtkX9H.png)

> [上图中的warning信息](https://blog.csdn.net/u011888840/article/details/107818360)是因为首次编译成功之后自动将`install`目录添加到了环境变量`AMENT_PREFIX_PATH`和`CMAKE_PREFIX_PATH`，而后手动删除了包括`install`、`build`以及`log`目录，导致重新做`colcon build`构建时产生warning，实际并不影响构建过程，可忽视。
>
> 如果不手动删除三个目录，直接修改后重新构建，就不会报此类错误
>
> ![](https://s2.loli.net/2024/07/12/iwceLabJVFDr3Mg.png)

### 2.2 Python版本

在`Code/ROS2/Test/Python/example_ws/src/example_py/example_py`下新建`node_04.py`，输入下面的代码

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class Node04(Node):
    """
    创建一个Node04节点，并在初始化时输出一个话
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = Node04("node_04")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

```

接着修改`setup.py`

```python
    entry_points={
        'console_scripts': [
            "node_02 = example_py.node_02:main",
            "node_04 = example_py.node_04:main"
        ],
    },
```

> 注意格式和结尾的`,`符号，`console_scripts`是个数组。

编译测试

```bash
colcon build --packages-select example_py
source install/setup.bash
ros2 run example_py node_04
```

![](https://s2.loli.net/2024/07/12/KfIjlXU75Hs9cLP.png)

## 3 Colcon使用进阶

### 3.1 ROS生态中的构建系统和构建工具

使用CMake（GCC或Makefile）和 Python Setup打包工具既然可以完成ROS2代码的编译，那为什么还需要Colcon呢？

#### 3.1.1 构建系统与构建工具

两者的区分点在于针对的对象不同，构建系统只对单包构建，而构建工具则能按照依赖关系顺次调用构建系统完成一系列功能包的构建。

ROS中用到的构建系统：`CMake`、`ament_cmake`、`catkin`、`Python setuptools`。

ROS中用到的构建工具：`colcon`、`catkin_make`、`catkin_make_isolated`、`catkin_tools`。

很明显colcon作为构建工具，可以通过调用`CMake`、`Python setuptools`完成构建。

#### 3.1.2 常见构建系统

##### 3.1.2.1 CMake

[CMake](https://cmake.org/) 是一个跨平台构建系统生成器，项目使用独立于平台的文件指定其生成过程，用户则通过使用CMake为其平台上的本机工具生成构建系统来构建项目。

通常用法有：`cmake`、`make`、`make intsall`

##### 3.1.2.2 Python setuptools

`setuptools`是Python包的常用打包工具。Python 包使用文件来描述依赖项，以及如何构建和安装内容。在ROS2中，功能包可以是“普通”Python包，而在ROS1中，任何Python功能都是从CMake文件触发setup.py进行打包。

通常的用法有：`python setup.py`

##### 3.1.2.3 catkin

[catkin](http://wiki.ros.org/catkin)基于CMake，并提供了一组方便的函数，使编写CMake包更容易。它自动生成 CMake 配置文件以及 pkg 配置文件。它还提供了注册不同类型测试的函数。

#### 3.1.3 常见构建工具

##### 3.1.3.1 catkin_make

该工具仅调用 CMake 一次，并使用 CMake 的函数在单个上下文中处理所有包。虽然这是一种有效的方法，因为所有包中的所有目标都可以并行化，但它具有明显的缺点。由于所有函数名称、目标和测试都共享一个命名空间，并且规模更大，这很容易导致冲突。

##### 3.1.3.2 colcon

![image-20220604133925270](https://fishros.com/d2lros2/humble/chapt2/advanced/3.Colcon%E4%BD%BF%E7%94%A8%E8%BF%9B%E9%98%B6/imgs/image-20220604133925270.png)

[colcon](http://colcon.readthedocs.io/)是一个命令行工具，用于改进构建，测试和使用多个软件包的工作流程。[colcon 文档](https://colcon.readthedocs.io/en/released/index.html)

##### 3.1.3.3 ament_tools

`ament_tools`由用于构建 ROS 2 包的独立 Python 3 包提供。它是为引导ROS 2项目而开发的，因此仅针对Python 3，并且可以在Linux，MacOS和Windows上运行。

`ament_tools`支持构建以下软件包：

- 带有`package.xml`文件的 ROS 2 包。
- 带有`package.xml`普通的 CMake 包。
- 没有清单文件的普通 CMake 包（从 CMake 文件中提取包名称和依赖项）。
- 带有`package.xml`文件的 Python 包。
- 没有清单文件的 Python 包（从`setup.py`文件中提取包名称和依赖项）。

### 3.2.Colcon构建进阶

#### 3.2.1 build参数

##### 3.2.1.1 构建指令

- `--packages-select` ，仅生成单个包（或选定的包）。
- `--packages-up-to`，构建选定的包，包括其依赖项。
- `--packages-above`，整个工作区，然后对其中一个包进行了更改。此指令将重构此包以及（递归地）依赖于此包的所有包。

##### 3.2.1.2 指定构建后安装的目录

可以通过 `--build-base`参数和`--install-base`，指定构建目录和安装目录。

##### 3.2.1.3 合并构建目录

`--merge-install` 选项用于将所有包的安装目录合并到一个共享的安装目录中，而不是每个包各自的安装目录。这样做的好处是可以减少冗余，简化环境变量的设置，并且在某些情况下可以提高性能。

##### 3.2.1.4 符号链接安装

启用`--symlink-install`后将不会把文拷贝到install目录，而是通过创建符号链接的方式。

##### 3.2.1.5 错误时继续安装

启用`--continue-on-error`，当发生错误的时候继续进行编译。

##### 3.2.1.6 CMake参数

`--cmake-args`，将任意参数传递给CMake。与其他选项匹配的参数必须以空格为前缀。

##### 3.2.1.7 控制构建线程

- `--executor EXECUTOR`，用于处理所有作业的执行程序。默认值是根据所有可用执行程序扩展的优先级选择的。要查看完整列表，请调用 `colcon extensions colcon_core.executor --verbose`。

  - `sequential` [`colcon-core`]

    一次处理一个包。

  - `parallel` [`colcon-parallel-executor`]

    处理多个作业**平行**.

- --parallel-workers NUMBER

  - 要并行处理的最大作业数。默认值为 [os.cpu_count()](https://docs.python.org/3/library/os.html#os.cpu_count) 给出的逻辑 CPU 内核数。

##### 3.2.1.8 开启构建日志

使用`--log-level`可以设置日志级别，比如`--log-level  info`。

## 4 ROS2节点发现与多机通信

在没有ROSMASTER的情况下，ROS2如何实现互相发现呢？

### 4.1 概述

ROS2用于通讯的默认中间件是DDS，在DDS中，不同逻辑网络共享物理网络的主要机制称为域(Domain) ID。同一域上的ROS2节点可以自由地相互发现并发送消息，而不同域上的ROS 2节点则不能。所有ROS2节点默认使用域ID为0。为了避免在同一网络上运行ROS 2的不同计算机组之间互相干扰，应为每组设置不同的域ID。

### 4.2 选择域ID (短版本)

下文解释了应该在ROS2中使用的域ID范围的推导。要跳过该背景知识并且只是选择一个安全的数字，只需选择一个介于0和101之间的安全的域ID (包括0和101)。

### 4.3 选择域ID (长版本)

DDS使用域ID计算将用于发现和通讯的UDP端口。有关如何计算端口的详细信息，请参见[这篇文章](http://dev.ros2.fishros.com/doc/Concepts/About-Domain-ID.html#domain-id-to-udp-port-calculator) 。我们知道在网络中，UDP端口是 无符号16位整型 ，因此可以分配的最大端口号是65535，通过公式计算，这意味着可以分配的最高域账号是232，而可以分配的最低域账号是0。

### 4.4 特定平台的约束

为了实现最大限度的兼容性，在选择域账号时应遵循一些特定于平台的附加约束。特别是，最好避免在操作系统的 临时端口范围中分配域ID，这避免了ROS2节点使用的端口与计算机上的其他网络服务之间可能的冲突。

以下是一些关于特定平台临时端口的提示。

**Linux**

默认情况下，linux内核使用端口32768-60999作为临时端口。这意味着域ID 0-101和215-232可以安全使用，而不会与临时端口发生冲突。临时端口范围可在Linux中通过在 `/proc/sys/net/ipv4/ip_local_port_range` 中设置自定义值进行配置。如果使用自定义临时端口范围，则可能需要相应地调整上述数字。

**macOS**

默认情况下，macOS上的临时端口范围为49152-65535。这意味着域ID 0-166可以安全使用，而不会与临时端口发生冲突。通过为 `net.inet.ip.portrange.first` 和 `net.inet.ip.portrange.last` 设置自定义sysctl值，临时端口范围可在macOS中配置。如果使用自定义临时端口范围，则可能需要相应地调整上述数字。

**Windows**

默认情况下，Windows上的临时端口范围为49152-65535。这意味着域ID 0-166可以安全使用，不会与临时端口发生冲突。临时的端口范围可通过 使用netsh 在窗口中配置。如果使用自定义临时端口范围，则可能需要相应地调整上述数字。

### 4.5 参与者约束

对于计算机上运行的每个ROS 2进程，将创建一个DDS "participant" 。由于每个DDS参与者占用计算机上的两个端口，因此在一台计算机上运行120个以上的ROS 2进程可能会溢出到其他域ID或临时端口。

为了解释原因，我们考虑域ID编号1和2。

- 域ID 1使用端口7650和7651进行多播。
- 域ID 2使用端口7900和7901进行多播。
- 在域ID 1中创建第一个进程 (第0个参与者) 时，端口7660和7661用于单播。
- 在域ID 1中创建第120个进程 (第119个参与者) 时，端口7898和7899用于单播。
- 在域ID 1中创建第121个进程 (第120个参与者) 时，端口7900和7901用于单播，并与域ID 2重叠。

如果已知计算机一次只能在一个域ID上，并且域ID足够低，那么创建比这更多的ROS 2进程是安全的。

在选择特定平台域 ID 范围顶部的域 ID 时，还有一个限制因素需要考虑。

例如，假设一台ID为101的Linux计算机:

- 计算机上的第0个ROS 2进程将连接到端口32650、32651、32660和32661。
- 计算机上的第1个ROS 2进程将连接到端口32650、32651、32662和32663。
- 计算机上的第53个ROS 2进程将连接到端口32650、32651、32766和32767。
- 计算机上的第54个ROS 2进程将连接到端口32650、32651、32768和32769，运行在临时端口范围内。

因此，在Linux上使用域ID为101时应创建的最大进程数为54。同样，在Linux上使用域ID为232时应创建的最大进程数为63，因为最大端口号为65535。

macOS和Windows的情况相似，尽管数字不同。在macOS和Windows上，当选择166 (范围顶部) 的域账号时，运行到临时端口范围之前，可以在计算机上创建的ROS 2进程的最大数量为120。

### 4.6 域ID到UDP端口号计算器

> 到[这里](http://dev.ros2.fishros.com/doc/Concepts/About-Domain-ID.html#domain-id-to-udp-port-calculator)试用

![img](https://fishros.com/d2lros2/humble/chapt2/advanced/4.ROS2%E8%8A%82%E7%82%B9%E5%8F%91%E7%8E%B0%E4%B8%8E%E5%A4%9A%E6%9C%BA%E9%80%9A%E4%BF%A1/imgs/8f2b8e7ee552ad5daaf1ba626de226fa.png)