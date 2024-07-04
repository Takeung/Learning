# Ubuntu22.04安装ROS2

## 1、确定Ubuntu和ROS版本的对应关系

[REP 2000 – ROS 2 Releases and Target Platforms (ROS.org)](https://www.ros.org/reps/rep-2000.html#rolling-ridley-june-2020-ongoing)

## 2. 配置及安装

官方安装文档链接：[Ubuntu (Debian) — ROS 2 Documentation: Humble documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### 2.1 设置locale

官方案例自然地使用英文locale，但是根据文档描述只要是支持UTF-8的locale都可以。

```
`sudo apt update && sudo apt install locales`
`sudo locale-gen en_US en_US.UTF-8`
`sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8`
`export LANG=en_US.UTF-8`
```

### 2.2 设置Ubuntu软件源

![](https://s2.loli.net/2024/07/04/E1TpWFVhJ8yRoKj.png)

#### 2.2.1 首先确认是否已经启用Universe源

使用如下命令检查:

```
apt-cache policy | grep universe
```

返回可能有若干行，但是应该包含如下内容：

```
500 http://us.archive.ubuntu.com/ubuntu jammy/universe amd64 Packages
    release v=22.04,o=Ubuntu,a=jammy,n=jammy,l=Ubuntu,c=universe,b=amd64
```

如果没有包含上述内容，那么输入如下命令：

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

如果已经正确包含上述信息，即可直接继续。

#### 2.2.2 添加ROS 2 apt 仓库

a) 添加证书

```
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

这一步如果遇到Failed to connect to raw.githubusercontent.com，请参考[源文档](https://blog.csdn.net/toopoo/article/details/127178416)解决。

b) 添加ros仓库

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.3 安装ros2包

```
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

你可能会遇到如下错误：

![](https://s2.loli.net/2024/07/04/QGSdX7sAnz2fPOw.png)

**解决方法**：先把 “/etc/apt/sources.list” 中的： http://cn.archive.ubuntu.com/ubuntu全部替换为可用的镜像，例如阿里云的：http://mirrors.aliyun.com/ubuntu/。然后使用

```
sudo apt --fix-broken install
```

命令修复未完成的安装即可。

如果已经预先修改好 “/etc/apt/sources.list” 中的源，可以将2.3节中一开始的命令重新执行一遍即可。

### 2.4 配置环境变量

```
source /opt/ros/humble/setup.bash
echo " source /opt/ros/humble/setup.bash" >> ~/.bashrc 
```

## 3 测试

打开第一个终端，启动一个数据的发布者节点：

```
ros2 run demo_nodes_cpp talker
```

应该看到

![](https://s2.loli.net/2024/07/04/lfWwLPxZr278N6H.png)

打开第二个终端，启动一个数据的订阅者节点：

```
ros2 run demo_nodes_py listener
```

应该能看到：

![](https://s2.loli.net/2024/07/04/7xnuBbiev4t3LZA.png)