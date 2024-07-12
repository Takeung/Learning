# ROS2实操入门

- [ROS2实操入门](#ros2实操入门)
  - [1 ROS2节点介绍](#1-ros2节点介绍)
    - [1.1 ROS2节点是什么](#11-ros2节点是什么)
    - [1.2 节点之间如何交互？](#12-节点之间如何交互)
    - [1.3 如何启动一个节点？](#13-如何启动一个节点)
    - [1.4 通过命令行界面查看节点信息](#14-通过命令行界面查看节点信息)
      - [1.4.1 ROS2命令行](#141-ros2命令行)
      - [1.4.2 节点相关的CLI](#142-节点相关的cli)
  - [2 ROS2工作包与工作空间](#2-ros2工作包与工作空间)
    - [2.1 工作空间](#21-工作空间)
    - [2.2 功能包是什么](#22-功能包是什么)
    - [2.3 功能包获取的两种方式](#23-功能包获取的两种方式)
      - [2.3.1 安装获取](#231-安装获取)
      - [2.3.2 手动编译获取](#232-手动编译获取)
    - [2.4 与功能包相关的指令 `ros2 pkg`](#24-与功能包相关的指令-ros2-pkg)
      - [2.4.1 创建功能包](#241-创建功能包)
      - [2.4.2 列出可执行文件](#242-列出可执行文件)
      - [2.4.3 列出所有的包](#243-列出所有的包)
      - [2.4.4 输出某个包所在路径的前缀](#244-输出某个包所在路径的前缀)
      - [2.4.5 列出包的清单描述文件](#245-列出包的清单描述文件)
  - [3 ROS2构建工具—`Colcon`](#3-ros2构建工具colcon)
    - [3.1 `Colcon`是个啥](#31-colcon是个啥)
    - [3.2 安装colcon](#32-安装colcon)
    - [3.3 编个东西测试一下](#33-编个东西测试一下)
    - [3.4 运行一个自己编的节点](#34-运行一个自己编的节点)
    - [3.5 本节学习指令](#35-本节学习指令)
      - [3.5.1 只编译一个包](#351-只编译一个包)
      - [3.5.2 不编译测试单元](#352-不编译测试单元)
      - [3.5.3 运行编译的包的测试](#353-运行编译的包的测试)
      - [3.5.4 允许通过更改src下的部分文件来改变install（重要）](#354-允许通过更改src下的部分文件来改变install重要)
  - [4 使用rclcpp编写节点](#4-使用rclcpp编写节点)
    - [4.1 创建工作空间和功能包](#41-创建工作空间和功能包)
      - [4.1.1 工作空间](#411-工作空间)
      - [4.1.2 创建`example_cpp`功能包](#412-创建example_cpp功能包)
    - [4.2 创建节点](#42-创建节点)
    - [4.3 编写代码](#43-编写代码)
      - [4.3.1 编写代码](#431-编写代码)
      - [4.3.2 修改CMakeLists](#432-修改cmakelists)
    - [4.4 编译运行节点](#44-编译运行节点)
      - [4.4.1 编译节点](#441-编译节点)
      - [4.4.2 source环境](#442-source环境)
      - [4.4.3 运行节点](#443-运行节点)
    - [4.5 测试](#45-测试)
  - [5 使用rclpy编写节点](#5-使用rclpy编写节点)
    - [5.1 创建Python功能包](#51-创建python功能包)
    - [5.2 编写程序](#52-编写程序)
    - [5.3 编译运行节点](#53-编译运行节点)
      - [5.3.1 编译节点](#531-编译节点)
      - [5.3.2 source环境](#532-source环境)
      - [5.3.3 运行节点](#533-运行节点)
    - [5.4 测试](#54-测试)
    - [6 总结](#6-总结)


## 1 ROS2节点介绍

### 1.1 ROS2节点是什么

ROS2中每个节点都只负责一个单独的模块化功能（比如一个节点负责控制车轮转动，一个节点负责从激光雷达获取数据、一个节点负责处理激光雷达的数据、一个节点负责定位等等）

![](https://s2.loli.net/2024/07/12/ydZH4PptxnlEi1Q.png)

### 1.2 节点之间如何交互？

以激光雷达举例，一个节点负责获取激光雷达的扫描数据，一个节点负责处理激光雷达数据，比如去除噪点。

那节点与节点之间就必须要通信，ROS2一共提供了四种通信方式:

- 话题-topics
- 服务-services
- 动作-Action
- 参数-parameters

这四种通信方式的用途和使用方法，可以通过官方提供的下图帮助理解![](https://s2.loli.net/2024/07/12/fg6iqmTbZwouCnx.gif)

### 1.3 如何启动一个节点？

使用指令：

```bash
ros2 run <package_name> <executable_name>
```

指令意义：启动`<package_name>`包下的`<executable_name>`节点。

**使用样例：**

```bash
ros2 run turtlesim turtlesim_node
```

运行之后可以看到一只小乌龟，接下来就可以试试下一节中提到的几个指令来查看节点信息和列表。

### 1.4 通过命令行界面查看节点信息

> ROS2命令行工具源码：[ros2/ros2cli: ROS 2 command line interface tools (github.com)](https://github.com/ros2/ros2cli)

#### 1.4.1 ROS2命令行

ROS2的CLI，就是和ROS2相关的命令行操作。

> CLI（Command-Line Interface）和GUI（Graphical User Interface）
>
> - GUI（Graphical User Interface）就是平常我们说的图形用户界面，大家用的Windows是就是可视化的，我们可以通过鼠标点击按钮等图形化交互完成任务。
> - CLI（Command-Line Interface）就是命令行界面了，我们所用的终端，黑框框就是命令行界面，没有图形化。

很久之前电脑还是没有图形化界面的，所有的交互都是通过命令行实现，就学习机器人而言，命令行操作相对于图形化优势更加明显。ROS2为我们提供了一系列指令，通过这些指令，可以实现对ROS2相关模块信息的获取、设置等操作。

#### 1.4.2 节点相关的CLI

运行节点（**常用**）

```bash
ros2 run <package_name> <executable_name>
```

查看节点列表（**常用**）：

```bash
ros2 node list
```

查看节点信息（**常用**）：

```bash
ros2 node info <node_name>
```

重映射节点名称

```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

运行节点时设置参数

```bash
ros2 run example_parameters_rclcpp parameters_basic --ros-args -p rcl_log_level:=10
```

## 2 ROS2工作包与工作空间

开发者想要运行一个节点时，可以使用以下命令

```
ros2 run 包名字 可执行文件名字
```

这里有个前提，要知道对应的可执行文件在哪个包，那问题就来了，想要找到某个包，该去哪里找？

**答案就是：工作空间**

> 注意：一个工作空间下可以有多个功能包，一个功能包可以有多个节点存在

### 2.1 工作空间

工作空间就是包含若干个功能包的目录，可以简单理解成一个文件夹，这个文件夹下有`src`。
所以一般可以通过以下指令新建一个工作空间

```bash
cd Code/ROS2/Test/Cpp
mkdir -p example_ws/src
```

### 2.2 功能包是什么

功能包可以理解为存放节点的地方，按照不同的编译方式，ROS2中的功能包可以分为三种类型。

- ament_python，适用于python程序
- cmake，适用于C++
- ament_cmake，适用于C++程序，是cmake的增强版

### 2.3 功能包获取的两种方式

#### 2.3.1 安装获取

安装一般使用

```bash
sudo apt install ros-<version>-package_name
```

安装获取会自动放置到系统目录，不用再次手动source。

#### 2.3.2 手动编译获取

手动编译相对麻烦一些，需要下载源码然后进行编译生成相关文件。

> 什么时候需要手动编译呢？
>
> 一般能直接安装的功能包都是作者编译好程序将可执行文件上传到仓库中，然后我们才能够通过`apt`进行安装，如果作者还没来得及测试上传，或者忘记了测试上传，就会找不到对应的包，这时候就需要手动编译安装了。
>
> 除此之外还有一种情况，就是当我们需要对包的源码进行修改时，也需要自行编译修改。

完成手动编译之后，**需要手动`source`工作空间的`install`目录**。

### 2.4 与功能包相关的指令 `ros2 pkg`

```bash
create       # Create a new ROS2 package
executables  # Output a list of package specific executables
list         # Output a list of available packages
prefix       # Output the prefix path of a package
xml          # Output the XML of the package manifest or a specific tag
```

#### 2.4.1 创建功能包

```bash
ros2 pkg create <package-name> --build-type {cmake,ament_cmake,ament_python} -- dependencies <依赖名字>
```

#### 2.4.2 列出可执行文件

列出所有

```bash
ros2 pkg executables
```

列出`turtlesim`功能包的所有可执行文件

```bash
ros2 pkg executables turtlesim
```

![](https://s2.loli.net/2024/07/12/7nMOpLfCg9yqYGB.png)

#### 2.4.3 列出所有的包

```bash
ros2 pkg list
```

#### 2.4.4 输出某个包所在路径的前缀

```bash
ros2 pkg prefix <package-name>
```

比如小乌龟

```bash
ros2 pkg prefix turtlesim
```

#### 2.4.5 列出包的清单描述文件

> 每一个功能包都有一个标配的`manifest.xml`文件，用于记录这个包的名字，构建工具，编译信息，拥有者，用途等信息。通过这个信息，就可以自动为该功能包安装依赖，构建时确定编译顺序等

查看小乌龟模拟器功能包的信息。

```bash
ros2 pkg xml turtlesim 
```

## 3 ROS2构建工具—`Colcon`

> - colcon官方文档 https://colcon.readthedocs.io/en/released/user/installation.html
> - ROS2官网文档 [https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)

### 3.1 `Colcon`是个啥

`colcon`其实是一个功能包构建工具，简而言之就是用来编译代码的，ROS2工作空间的编译就是用`colcon`。

### 3.2 安装colcon

ROS2默认是没有安装`colcon`的，需要开发者手动安装。

```bash
sudo apt-get install python3-colcon-common-extensions
```

安装完成后，打开终端输入`colcon`即可看到其使用方法。

### 3.3 编个东西测试一下

1. 创建一个工作区文件夹`colcon_test_ws`

   ```bash
   mkdir colcon_test_ws
   cd colcon_test_ws
   ```

2. 下载个ROS2示例源码测试一下

   ```bash
   git clone https://github.com/ros2/examples src/examples -b humble
   ```

   > 如果这步克隆错误，可能需要一个[梯子](https://portal.shadowsocks.nz/aff.php?aff=41638)翻个墙

3. 编译工程

   ```bash
   colcon build
   ```

   ![](https://s2.loli.net/2024/07/12/vqYQyFVkK9TJXnp.png)

   > 如果在编译中遇到`Setuptools DeprecationWarning: setup.py install is deprecated.`这个警告，可以通过更新setuptools解决。详细操作见社区帖子：https://fishros.org.cn/forum/topic/254/

4. 编完之后的目录结构

   构建完成后，在`src`同级目录我们应该会看到 `build` 、 `install` 和 `log` 目录:

   ```bash
   .
   ├── build
   ├── install
   ├── log
   └── src
   
   4 directories, 0 files
   ```

   - `build` 目录存储的是中间文件。对于每个包，将创建一个子文件夹，在其中调用例如CMake。
   - `install` 目录是每个软件包将安装到的位置。默认情况下，每个包都将安装到单独的子目录中。
   - `log` 目录包含有关每个colcon调用的各种日志信息。

### 3.4 运行一个自己编的节点

1. 打开一个终端进入刚刚创建的工作空间，先`source` 一下资源

   ```bash
   source install/setup.bash
   ```

2. 运行一个订者节点，你将看不到任何打印，因为没有发布者

   ```bash
   ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
   ```

3. 打开一个新的终端，先`source`，再运行一个发行者节点

   ```bash
   source install/setup.bash
   ros2 run examples_rclcpp_minimal_publisher publisher_member_function
   ```

   ![](https://s2.loli.net/2024/07/12/796ElizdPIY1ZXV.png)

### 3.5 本节学习指令

#### 3.5.1 只编译一个包

```bash
colcon build --packages-select YOUR_PKG_NAME 
```

#### 3.5.2 不编译测试单元

```bash
colcon build --packages-select YOUR_PKG_NAME --cmake-args -DBUILD_TESTING=0
```

#### 3.5.3 运行编译的包的测试

```bash
colcon test
```

#### 3.5.4 允许通过更改src下的部分文件来改变install（重要）

每次调整 `python` 脚本时都不必重新`build`了

```bash
colcon build --symlink-install
```

## 4 使用rclcpp编写节点

节点需要存在于功能包当中、功能包需要存在于工作空间当中。所以要想创建节点，就要先创建一个工作空间，再创建功能包。

### 4.1 创建工作空间和功能包

#### 4.1.1 工作空间

工作空间就是文件夹，所以很简单。

```bash
cd Code/ROS2/Test/Cpp
mkdir -p example_ws/src/
```

#### 4.1.2 创建`example_cpp`功能包

创建`example_cpp`功能包，使用`ament-cmake`作为编译类型，并为其添加`rclcpp`依赖。

```bash
cd example_ws/src
ros2 pkg create example_cpp --build-type ament_cmake --dependencies rclcpp
```

> - pkg create：创建包
> - --build-type：指定该包的编译类型，共有三个可选项`ament_python`、`ament_cmake`、`cmake`
> - --dependencies：指定该功能包的依赖，这里是`rclcpp`

打开终端，进入`example_ws/src`运行上面的指令，创建完成后的目录结构如下：

```bash
.
└── src
    └── example_cpp
        ├── CMakeLists.txt
        ├── include
        │   └── example_cpp
        ├── package.xml
        └── src

5 directories, 2 files
```

### 4.2 创建节点

接着在`example_cpp/src`下创建一个`node_01.cpp`文件，创建完成后的目录结构如下：

![](https://s2.loli.net/2024/07/12/L7JrhsOmPuv2VRe.png)

### 4.3 编写代码

#### 4.3.1 编写代码

在`node_01.cpp`中输入以下内容。

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    /* 初始化rclcpp  */
    rclcpp::init(argc, argv);
    /*产生一个node_01的节点*/
    auto node = std::make_shared<rclcpp::Node>("node_01");
    // 打印一句自我介绍
    RCLCPP_INFO(node->get_logger(), "node_01节点已经启动.");
    /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
    /* 停止运行 */
    rclcpp::shutdown();
    return 0;
}
```

#### 4.3.2 修改CMakeLists

编码完成后需要修改CMakeLists.txt：将其添加为可执行文件，并使用`install`指令将其安装到`install`目录。

在`CMakeLists.txt`最后一行`ament_package()`之前加入下述代码，目的是让编译器编译`node_01`这个文件。

```cmake
# 添加可执行文件
add_executable(node_01 src/node_01.cpp)
ament_target_dependencies(node_01 rclcpp)

# 安装可执行文件
install(TARGETS
  node_01
  DESTINATION lib/${PROJECT_NAME}
)
```

### 4.4 编译运行节点

在`example_ws`下依次输入下述命令

#### 4.4.1 编译节点

```bash
colcon build
```

> 构建时路径中不能包含“++”等特殊符号，会导致[正则编译失败](https://www.jianshu.com/p/73dd7ee26047)![](https://s2.loli.net/2024/07/12/j5f8TwZ2KRStCyE.png)

#### 4.4.2 source环境

```bash
source install/setup.bash
```

#### 4.4.3 运行节点

```bash
ros2 run example_cpp node_01
```

![](https://s2.loli.net/2024/07/12/WBK7CY5aZ6odSgA.png)

### 4.5 测试

当节点运行起来后，可以新开控制台使用`ros2 node list `指令来查看现有的节点：

![](https://s2.loli.net/2024/07/12/LcsjimXyo3hqNGr.png)

## 5 使用rclpy编写节点

### 5.1 创建Python功能包

创建一个名为`example_py` 的python功能包。

```bash
cd Code/ROS2/Test/Python
mkdir -p example_ws/src
cd example_ws/src
ros2 pkg create example_py --build-type ament_python --dependencies rclpy
```

创建完成后的目录结构

```bash
.
├── example_py
│   └── __init__.py
├── package.xml
├── resource
│   └── example_py
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

3 directories, 8 files
```

### 5.2 编写程序

编写ROS2节点的一般步骤

```markdown
1. 导入库文件
2. 初始化客户端库
3. 新建节点
4. spin循环节点
5. 关闭客户端库
```

在`example_py/example_py`下创建`node_02.py`，代码如下。

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    """
    ros2运行该节点的入口函数
    编写ROS2节点的一般步骤
    1. 导入库文件
    2. 初始化客户端库
    3. 新建节点对象
    4. spin循环节点
    5. 关闭客户端库
    """
    rclpy.init(args=args) # 初始化rclpy
    node = Node("node_02")  # 新建一个节点
    node.get_logger().info("大家好，我是node_02.")
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
```

代码编写完成后保存，接着修改`example_py/setup.py`。

```python
    entry_points={
        'console_scripts': [
            "node_02 = example_py.node_02:main"
        ],
    },
)
```

上述配置是用于声明一个ROS2节点，声明后使用`colcon build`就能检测到，从而将其添加到`install`目录下。

### 5.3 编译运行节点

#### 5.3.1 编译节点

```bash
cd ~/Code/ROS2/Test/Python/example_ws/
colcon build
```

> ```
> --- stderr: example_py                   
> /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
>   warnings.warn(
> ---
> ```
>
> 如果在编译中看到上述错误没关系，不影响使用，ros2官方正在修复。 错误原因是setuptools版本太高造成，使用下面的指令可以进行版本的回退。
>
> ```
> sudo pip install setuptools==58.2.0 --upgrade
> ```

#### 5.3.2 source环境

```bash
source install/setup.bash
```

#### 5.3.3 运行节点

```bash
ros2 run example_py node_02
```

![](https://s2.loli.net/2024/07/12/34DjcywHgXEJrx1.png)

### 5.4 测试

当节点运行起来后，可以新开控制台使用`ros2 node list `指令来查看现有的节点：

![](https://s2.loli.net/2024/07/12/IOfjEsZukPApYh5.png)

### 6 总结

本文初步介绍了如何在C++和Python的工作空间中通过最简方式编写ROS2节点，但在实际应用中需要解决的问题更为复杂，所以面向对象的编写方式应用范围更加广泛，相应的进阶用法同样需要掌握。