# ROS2节点编译之C++入门

- [ROS2节点编译之C++入门](#ros2节点编译之c入门)
	- [1. 用g++编译ROS2的C++节点](#1-用g编译ros2的c节点)
		- [1.1 编写节点](#11-编写节点)
		- [1.2 编译](#12-编译)
		- [1.3. 运行节点](#13-运行节点)
	- [2. 使用make编译ROS2节点](#2-使用make编译ros2节点)
		- [2.1. 安装make](#21-安装make)
		- [2.2. 编写Makefile](#22-编写makefile)
		- [2.3. 编译](#23-编译)
		- [2.4. 运行测试](#24-运行测试)
	- [3. 使用CMakeLists.txt编译ROS2节点](#3-使用cmakeliststxt编译ros2节点)
		- [3.1. 安装Cmake](#31-安装cmake)
		- [3.2. 新建CMakeLists.txt](#32-新建cmakeliststxt)
		- [3.3. 编译代码](#33-编译代码)
	- [4. CMake依赖查找流程](#4-cmake依赖查找流程)
		- [4.1. 优化CMakeList.txt](#41-优化cmakelisttxt)
		- [4.2. find\_package查找路径](#42-find_package查找路径)
		- [4.3. 总结](#43-总结)


## 1. 用g++编译ROS2的C++节点

### 1.1 编写节点

编写一个ROS2的C++节点非常简单，只需三行代码即可完成。

打开终端，创建`Test/C++/d2lros2`目录，用`VSCode`打开`d2lros2`目录。

```bash
mkdir -p Test/C++/d2lros2
code d2lros2
```

接着在该路径下新建`first_ros2_node.cpp`，并输入下面的代码

```c++
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // 调用rclcpp的初始化函数
    rclcpp::init(argc, argv);
    // 调用rclcpp的循环运行我们创建的first_node节点
    rclcpp::spin(std::make_shared<rclcpp::Node>("first_node"));
    return 0;
}
```

### 1.2 编译

接着使用g++来编译`first_ros2_node`节点。正常的话一定会报错。

```bash
g++ first_ros2_node.cpp
```

报错内容如下：

```bash
$ g++ first_ros2_node.cpp 
first_ros2_node.cpp:3:10: fatal error: rclcpp/rclcpp.hpp: No such file or directory
    3 | #include "rclcpp/rclcpp.hpp"
      |          ^~~~~~~~~~~~~~~~~~~
compilation terminated.
```

**错误的原因是我们在代码里包含了"rclcpp/rclcpp.hpp"头文件，但是g++找不到这个头文件，解决方法就是告诉g++这个头文件的目录。**

首先我们要找到这个头文件在哪里，这个头文件是ROS2的客户端库，其地址肯定在ROS2的安装目录下，即`/opt/ros/humble/include/rclcpp`。

```bash
cd /opt/ros/humble/include/rclcpp
ls rclcpp/* | grep rclcpp.h
```

> ls指令列出命令 | grep rclcpp.h 是对列出的结果进行过滤，只显示包含`rclcpp.h`的行。

使用上面的指令，可以看到这个文件确实在这里。

![](https://s2.loli.net/2024/07/11/Mbzytx4Z1lw7kdP.png)

接着我们可以用-I（大写i）来为g++指定这个目录，然后再次运行，你会发现依然报错

```
g++ first_ros2_node.cpp -I /opt/ros/humble/include/rclcpp/
```

报错如下

```bash
$ g++ first_ros2_node.cpp -I/opt/ros/humble/include/rclcpp/ 
In file included from /opt/ros/humble/include/rclcpp/rclcpp/executors/multi_threaded_executor.hpp:25,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executors.hpp:21,
                 from /opt/ros/humble/include/rclcpp/rclcpp/rclcpp.hpp:155,
                 from first_ros2_node.cpp:3:
/opt/ros/humble/include/rclcpp/rclcpp/executor.hpp:30:10: fatal error: rcl/guard_condition.h: No such file or directory
   30 | #include "rcl/guard_condition.h"
      |          ^~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.Copy to clipboardErrorCopied
```

虽然错误有些不一样，但是核心的文件都是一样的，错误信息提示：在`/opt/ros/humble/include/rclcpp/rclcpp/executors/multi_threaded_executor.hpp:25`这个位置，包含了`rcl/guard_condition.h`，但是找不到这个头文件。解决方案也是相同的，`rcl/guard_condition.h`所在的路径是`/opt/ros/humble/include/rcl/`，再次指定后运行。

```bash
g++ first_ros2_node.cpp -I /opt/ros/humble/include/rclcpp/ -I /opt/ros/humble/include/rcl/
```

同样会再次遇到相同错误，因为头文件的包含是类似于套娃形式的，一层层加下去，总有终点，最终

```bash
g++ first_ros2_node.cpp \
-I/opt/ros/humble/include/rclcpp/ \
-I /opt/ros/humble/include/rcl/ \
-I /opt/ros/humble/include/rcutils/ \
-I /opt/ros/humble/include/rmw \
-I /opt/ros/humble/include/rcl_yaml_param_parser/ \
-I /opt/ros/humble/include/rosidl_runtime_c \
-I /opt/ros/humble/include/rosidl_typesupport_interface \
-I /opt/ros/humble/include/rcpputils \
-I /opt/ros/humble/include/builtin_interfaces \
-I /opt/ros/humble/include/rosidl_runtime_cpp \
-I /opt/ros/humble/include/tracetools \
-I /opt/ros/humble/include/rcl_interfaces \
-I /opt/ros/humble/include/libstatistics_collector \
-I /opt/ros/humble/include/statistics_msgs
```

运行完上面这段代码，你会发现报的错误变了。

```bash
/usr/bin/ld: /tmp/ccoA8hho.o: in function `main':
first_ros2_node.cpp:(.text+0x37): undefined reference to `rcutils_get_default_allocator'
/usr/bin/ld: first_ros2_node.cpp:(.text+0x5c): undefined reference to `rclcpp::InitOptions::InitOptions(rcutils_allocator_s)'
/usr/bin/ld: first_ros2_node.cpp:(.text+0x7d): undefined reference to `rclcpp::init(int, char const* const*, rclcpp::InitOptions const&, rclcpp::SignalHandlerOptions)'
/usr/bin/ld: first_ros2_node.cpp:(.text+0x89): undefined reference to `rclcpp::InitOptions::~InitOptions()'
/usr/bin/ld: first_ros2_node.cpp:(.text+0xb1): undefined reference to `rclcpp::spin(std::shared_ptr<rclcpp::Node>)'
/usr/bin/ld: first_ros2_node.cpp:(.text+0xe9): undefined reference to `rclcpp::InitOptions::~InitOptions()'
/usr/bin/ld: /tmp/ccoA8hho.o: in function `void __gnu_cxx::new_allocator<rclcpp::Node>::construct<rclcpp::Node, char const (&) [11]>(rclcpp::Node*, char const (&) [11])':
first_ros2_node.cpp:(.text._ZN9__gnu_cxx13new_allocatorIN6rclcpp4NodeEE9constructIS2_JRA11_KcEEEvPT_DpOT0_[_ZN9__gnu_cxx13new_allocatorIN6rclcpp4NodeEE9constructIS2_JRA11_KcEEEvPT_DpOT0_]+0x86): undefined reference to `rcutils_get_default_allocator'
/usr/bin/ld: first_ros2_node.cpp:(.text._ZN9__gnu_cxx13new_allocatorIN6rclcpp4NodeEE9constructIS2_JRA11_KcEEEvPT_DpOT0_[_ZN9__gnu_cxx13new_allocatorIN6rclcpp4NodeEE9constructIS2_JRA11_KcEEEvPT_DpOT0_]+0xb7): undefined reference to `rclcpp::NodeOptions::NodeOptions(rcutils_allocator_s)'
/usr/bin/ld: first_ros2_node.cpp:(.text._ZN9__gnu_cxx13new_allocatorIN6rclcpp4NodeEE9constructIS2_JRA11_KcEEEvPT_DpOT0_[_ZN9__gnu_cxx13new_allocatorIN6rclcpp4NodeEE9constructIS2_JRA11_KcEEEvPT_DpOT0_]+0xe7): undefined reference to `rclcpp::Node::Node(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, rclcpp::NodeOptions const&)'
collect2: error: ld returned 1 exit statusCopy to clipboardErrorCopied
```

**错误变成了`undefined reference to xxxxx`**，原因在于g++找不到库文件，解决方法就是定位到库文件的位置，并通过`-L`参数指定库目录，`-l`（小写L）指定库的名字。

ROS2相关的库的地址都在`/opt/ros/humble/lib`下，可以使用下面的指令看到`rclcpp`的动态链接库。

```bash
ls /opt/ros/humble/lib | grep rclcpp
```

![](https://s2.loli.net/2024/07/11/XFj24GSAfQxhbgm.png)

**指定库目录和使用的库后的终极命令**

```bash
g++ first_ros2_node.cpp \
-I/opt/ros/humble/include/rclcpp/ \
-I /opt/ros/humble/include/rcl/ \
-I /opt/ros/humble/include/rcutils/ \
-I /opt/ros/humble/include/rmw \
-I /opt/ros/humble/include/rcl_yaml_param_parser/ \
-I /opt/ros/humble/include/rosidl_runtime_c \
-I /opt/ros/humble/include/rosidl_typesupport_interface \
-I /opt/ros/humble/include/rcpputils \
-I /opt/ros/humble/include/builtin_interfaces \
-I /opt/ros/humble/include/rosidl_runtime_cpp \
-I /opt/ros/humble/include/tracetools \
-I /opt/ros/humble/include/rcl_interfaces \
-I /opt/ros/humble/include/libstatistics_collector \
-I /opt/ros/humble/include/statistics_msgs \
-L /opt/ros/humble/lib/ \
-lrclcpp -lrcutils
```

运行后，就没有任何报错了，并且在当前目录下会多出一个`a.out`，这个就是可执行文件。

> 可以在g++指令后面添加 `-o 名字` ，比如 `-o first_node`

### 1.3. 运行节点

执行代码

```bash
./a.out
```

打开新的终端，使用`ros2 node list`查看正在运行的节点，是否有`first_node`。

![](https://s2.loli.net/2024/07/11/BJHcQwDf3exKqEa.png)

## 2. 使用make编译ROS2节点

用g++编译节点无比的麻烦，为此先行者们发明了make的**批处理**工具，可以将g++的指令写成脚本完成操作。

### 2.1. 安装make

```bash
sudo apt install make
```

### 2.2. 编写Makefile

在`Test/C++/d2lros2`下新建`Makefile`，然后将上面的g++编译指令用下面的形式写到Makefile里。

```makefile
build:
	g++ first_ros2_node.cpp \
	-I/opt/ros/humble/include/rclcpp/ \
	-I /opt/ros/humble/include/rcl/ \
	-I /opt/ros/humble/include/rcutils/ \
	-I /opt/ros/humble/include/rmw \
	-I /opt/ros/humble/include/rcl_yaml_param_parser/ \
	-I /opt/ros/humble/include/rosidl_runtime_c \
	-I /opt/ros/humble/include/rosidl_typesupport_interface \
	-I /opt/ros/humble/include/rcpputils \
	-I /opt/ros/humble/include/builtin_interfaces \
	-I /opt/ros/humble/include/rosidl_runtime_cpp \
	-I /opt/ros/humble/include/tracetools \
	-I /opt/ros/humble/include/rcl_interfaces \
	-I /opt/ros/humble/include/libstatistics_collector \
	-I /opt/ros/humble/include/statistics_msgs \
	-L /opt/ros/humble/lib/ \
	-lrclcpp -lrcutils \
	-o first_node
    
# 删掉first_node
clean:
	rm first_node
```

> 注意拷贝过去后需要完成空格与TAB数量的转换。

### 2.3. 编译

在Makefile同级目录输入

```bash
make build
```

![](https://s2.loli.net/2024/07/11/bnZu9z7iYQaCAd6.png)

可以看到make指令调用了脚本里的build下的指令，对代码进行了编译。同级目录下也产生了first_node可执行文件（绿色代表可执行）。

使用`make clean`指令即可删掉`first_node`节点。

![](https://s2.loli.net/2024/07/11/sKWClTFgYOwM8PI.png)

### 2.4. 运行测试

```bash
./first_node
```

新开终端

```bash
ros2 node list
```

![](https://s2.loli.net/2024/07/11/BJHcQwDf3exKqEa.png)

## 3. 使用CMakeLists.txt编译ROS2节点

虽然通过make调用Makefile编译代码非常的方便，但是还是需要手写gcc指令，而**使用cmake则可以通过调用CMakeLists.txt直接生成Makefile。**

### 3.1. 安装Cmake

```bash
sudo apt install cmake
```

### 3.2. 新建CMakeLists.txt

在`Test/C++/d2lros2`新建`CMakeLists.txt`，输入下面内容。

```cmake
cmake_minimum_required(VERSION 3.22)

project(first_node)

#include_directories 添加特定的头文件搜索路径 ，相当于指定g++编译器的-I参数
include_directories(/opt/ros/humble/include/rclcpp/)
include_directories(/opt/ros/humble/include/rcl/)
include_directories(/opt/ros/humble/include/rcutils/)
include_directories(/opt/ros/humble/include/rcl_yaml_param_parser/)
include_directories(/opt/ros/humble/include/rosidl_runtime_c/)
include_directories(/opt/ros/humble/include/rosidl_typesupport_interface/)
include_directories(/opt/ros/humble/include/rcpputils/)
include_directories(/opt/ros/humble/include/builtin_interfaces/)
include_directories(/opt/ros/humble/include/rmw/)
include_directories(/opt/ros/humble/include/rosidl_runtime_cpp/)
include_directories(/opt/ros/humble/include/tracetools/)
include_directories(/opt/ros/humble/include/rcl_interfaces/)
include_directories(/opt/ros/humble/include/libstatistics_collector/)
include_directories(/opt/ros/humble/include/statistics_msgs/)

# link_directories - 向工程添加多个特定的库文件搜索路径，相当于指定g++编译器的-L参数
link_directories(/opt/ros/humble/lib/)

# add_executable - 生成first_node可执行文件
add_executable(first_node first_ros2_node.cpp)

# target_link_libraries - 为first_node(目标) 添加需要动态链接库，相同于指定g++编译器-l参数
# 下面的语句代替 -lrclcpp -lrcutils
target_link_libraries(first_node rclcpp rcutils)
```

### 3.3. 编译代码

开发者一般会创建一个新的目录`build`，运行`cmake`并进行编译，这样的好处是不会显得那么乱。

```bash
mkdir build
cd build
```

创建好文件夹，接着运行`cmake`指令，`..`代表到上级目录找`CMakeLists.txt`。

```bash
cmake ..
```

运行完`cmake`就可以在`build`目录下看到`cmake`自动生成的`Makefile`了，接着就可以运行`make`指令进行编译

```bash
make
```

运行完上面的指令，就可以在`build`目录下发现`first_node`节点了。

![](https://s2.loli.net/2024/07/11/LfvGpEXcqIZ5FsP.png)

## 4. CMake依赖查找流程

虽然`cmake`编译成功了，但是`CMakeLists.txt`的内容依然非常臃肿，需要将其进一步的简化。

### 4.1. 优化CMakeList.txt

将上面的`CmakLists.txt`改成下面的样子

```cmake
cmake_minimum_required(VERSION 3.22)
project(first_node)

find_package(rclcpp REQUIRED)
add_executable(first_node first_ros2_node.cpp)
target_link_libraries(first_node rclcpp::rclcpp)
```

接着继续生成和编译

```bash
cmake ..
make
```

![](https://s2.loli.net/2024/07/11/zQf2K5h1as7twZU.png)

那么，为什么可以浓缩成那么短的几句指令呢？

### 4.2. find_package查找路径

find_package查找路径对应的环境变量如下

```cmake
<package>_DIR
CMAKE_PREFIX_PATH
CMAKE_FRAMEWORK_PATH
CMAKE_APPBUNDLE_PATH
PATH
```

打开终端，输入指令：

```bash
echo $PATH
```

结果

```bash
PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
```

观察`PATH`变量，会发现`/opt/ros/humble/bin`赫然在其中，`PATH`中的路径如果以`bin`或`sbin`结尾，则自动回退到上一级目录，接着检查这些目录下的

```bash
<prefix>/(lib/<arch>|lib|share)/cmake/<name>*/          (U)
<prefix>/(lib/<arch>|lib|share)/<name>*/                (U)
<prefix>/(lib/<arch>|lib|share)/<name>*/(cmake|CMake)/  (U)
```

cmake找到这些目录后，会开始依次找`<package>Config.cmake`或`Find<package>.cmake`文件。找到后即可执行该文件并生成相关链接信息。打开`/opt/ros/humble/share/rclcpp/cmake`则会发现`rclcppConfig.cmake`就在其中。

### 4.3. 总结

本节主要介绍C++编译工具`cmake`及其路径查找规则。
