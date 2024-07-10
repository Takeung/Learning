# VirtualBox入门使用与进阶配置

## VirtualBox 是什么

我的日常工作离不开 VirtualBox，因为习惯于在 Linux 环境下工作，所以电脑上只保留了一个 Linux 系统。我会用 VirtualBox 来解决一些特殊情况下碰到的问题。

比如开发微信小程序，需要通过官方提供的「微信开发者工具」来实时打包并预览效果，但这个软件只支持 Windows 和 macOS，我可以用 VirtualBox 虚拟一台 Windows 系统来解决这个问题。另外对于一些依赖于分布式架构的软件或功能，比如要测试 MySQL 主从备份或是微服务架构，VirtualBox 就更加得心应手，可以快速模拟出多台机器让我使用和测试。

除了以上专业应用，我在另外一些场景下也会使用 VirtualBox。比如想尝试一款来路不明的软件，又有点担心其中包含了有恶意程序，用 VirtualBox 创建一个隔离的系统环境，可以避免主机系统遭受损失。以及我想尝试体验不同的操作系统时，VirtualBox 可以让我不用放弃当前正在使用的系统和数据，尝试失败了也不会带来任何损失，可以节省大量时间和精力。针对Linux 系统安装流程可以参看这几篇文章：

[Arch Linux 系统安装流程](https://www.zzxworld.com/posts/install_arch_linux_on_virtual_box)*Arch Linux 系统的安装步骤和流程记录。*

[Debian 11 系统安装流程](https://www.zzxworld.com/posts/debian_linux_install_guide)*Debian 11 Linux 系统的安装步骤和流程记录。*

[Ubuntu 22.04 LTS 系统安装流程](https://www.zzxworld.com/posts/ubuntu-jammy-jellyfish-linux-install-guide)*Ubuntu 22.04 LTS Linux 长期支持版的安装步骤和流程记录。*

这几个 Linux 系统的尝试和体验都是在 VirtualBox 上完成的。结合上面的使用场景，可以这样来理解 VirtualBox 的概念和用途：

> VirtualBox 这种虚拟机工具以软件的方式模拟出完善的电脑硬件环境，这个模拟出的硬件环境能正常安装各种操作系统，并能正常操作和使用。

通过 VirtualBox 模拟出的虚拟电脑并不是任何操作系统都能正常使用，具体要参考这份列表：

[VirtualBox Guest operation systems](https://www.virtualbox.org/wiki/Guest_OSes)*VirtualBox 虚拟环境下支持安装的操作系统列表。*

根据这个列表里面提供的信息，像 Windows 98/SE/ME 虽然能用，但没有任何性能优化，使用起来会很卡。macOS 则是完全不支持。总的来说，主流的 Windows 和 Linux 发行版本都没有问题。

## 下载安装 VirtualBox

VirtualBox 是一个跨平台软件，在 Windows 和 Linux 系统上都可以安装使用。至于 macOS，上面虽然说不支持在模拟环境运行 macOS，但支持在 macOS 上安装 VirtualBox 并虚拟 Windows 和 Linux 系统。

对于 Windows 和 macOS 系统，VirtualBox 提供了交互式的界面化安装程序，官方下载地址如下：

[Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)*VirtualBox 安装包官方下载地址。*

页面打开后，点击与当前系统对应的下载链接即可。安装文件不大，100M 左右。完成下载后双击运行程序，即可开始按照步骤一步步完成安装。

对于 Linux 系统，可以选择通过命令来安装，更加快捷和方便。比如我使用的 Arch Linux，就是使用这条命令:

```shell
sudo pacman -S virtualbox
```

不同的 Linux 系统使用的包管理命令不一样，具体可以参考官方的这个页面:

[Download for Linux](https://www.virtualbox.org/wiki/Linux_Downloads)*VirtualBox 在 Linux 系统上的安装说明。*

## 创建虚拟机

安装完成后，在应用程序里会看到 VirtualBox 的图标。点击后启动程序，映入眼帘的是 VirtualBox 的主界面。

![主界面](https://www.zzxworld.com/resource/posts/202206/1654871351.2585.png)

首次使用，直接把目光聚焦到「新建」这个按钮上，点一下，就打开了「新建虚拟电脑」的弹窗。

![新建虚拟电脑界面](https://www.zzxworld.com/resource/posts/202206/1654871375.1918.png)

这个界面有一个文本框和三个选择框，稍微介绍一下：

- 「名称」文本框自己随便输入，表示给当前要创建的虚拟电脑取个名字。我的习惯是使用要安装的系统名称和版本，这样方便后面识别和使用。
- 「文件夹」选择框可以指定虚拟电脑相关文件要保存的位置。通常保持默认。
- 「类型」选择框指定要虚拟电脑安装的操作系统类型。这个根据实际情况来选择即可，如果准备安装 Windows 系统，就选择 Windows；要安装 Linux 类的系统就选择 Linux。
- 「版本」选择框就是指定更细节的系统信息。比如 Windows 系统下既有 XP，2000，10，11 等各种版本，而且还有 32 位和 64 位之分。所以根据自己要安装的版本来选择。

完成上面的输入和选择后，点击下一步，给虚拟电脑分配内存大小。

![指定虚拟电脑内存大小](https://www.zzxworld.com/resource/posts/202206/1654871405.4136.png)

内存大小会影响虚拟系统的运行速度，所以根据主机可用内存的大小，尽量多分配一些。如果是安装服务器版本的 Linux 系统，建议最小 1024 MB。如果是有桌面环境的操作系统， 建议最小 4096 MB，至少 2048 MB。

完成内存设置后，继续点击下一步，进入虚拟硬盘的配置界面。

![指定虚拟电脑硬盘位置](https://www.zzxworld.com/resource/posts/202206/1654871425.4261.png)

这里通常保持「现在创建虚拟硬盘」的默认选项。然后点击创建按钮继续，进入选择虚拟硬盘文件类型的界面。

![选择虚拟硬盘文件类型](https://www.zzxworld.com/resource/posts/202206/1654871453.4235.png)

可以看到 VirtualBox 提供了三种虚拟硬盘类型：

- **VDI**: 这是 VirtualBox 专用的虚拟硬盘类型。
- **VHD**: 这是微软制定的一种虚拟硬盘格式。
- **VMDK**: 这是 VMware 软件的虚拟硬盘格式。

关于这三种格式的选择，如果创建的虚拟磁盘文件存在和其他虚拟机共用的情况，就要根据实际情况来选择。比如想把创建的虚拟机磁盘文件发送给一位朋友直接使用，而他用的是 VMware 这个虚拟机软件，那就可以选择 **VMDK** 格式。不过就我使用虚拟机的经历来看，从没碰到过这种情况。所以保持默认，选择 **VDI** 即可。点击下一步，来到虚拟硬盘大小分配方式选择界面。

![选择虚拟硬盘大小分配类型](https://www.zzxworld.com/resource/posts/202206/1654871871.2778.png)

这个界面有两个选项，界面上也提供了有具体的说明，总结一下两者的使用特征：

- **动态分配**：表示在使用虚拟电脑时，虚拟磁盘的空间根据使用情况自动增加。具有空间利用率的优势。
- **固定大小**：表示指定创建时就占用固定大小，即便是在虚拟电脑中没有使用到。具有使用速度上的优势。

我的选择建议是看自己主机的硬盘大小，如果是 TB 级别的，可以考虑固定大小，反正有的是空间。对于我自己来说，主机硬盘只有 512GB，磁盘空间可以说是寸土寸金，所以会青睐于动态分配。

选择好后继续点下一步，开始指定磁盘大小。

![指定磁盘大小](https://www.zzxworld.com/resource/posts/202206/1654871485.2469.png)

此处同样需要根据准备在虚拟机上安装的系统来选择。如果是准备安装 Linux，最小建议 10GB，稳妥建议 20GB。如果是 Windows 系统，建议大小翻倍。完成设置后点击创建按钮，就完成了虚拟电脑的创建。

## 安装系统

虚拟电脑创建完成后会回到 VirtualBox 主界面。目前这个虚拟电脑就像是一台刚出厂的新机器，双击左侧虚拟电脑的名称，或是选中后点击右侧工具栏的启动图标都可以启动它。只是目前它啥都还不能做，我们需要给它安装一个操作系统。这里我将以 Ubuntu 系统为例，介绍一下大致的系统安装过程。

首先打开我前两天刚发布的一篇介绍 Ubuntu 系统安装的文章：

[Ubuntu Linux 系统安装流程](https://www.zzxworld.com/posts/ubuntu-jammy-jellyfish-linux-install-guide)*Ubuntu 22.04 Linux 系统详细的图解安装过程。*

定位到这篇文章的「下载安装映像」部分，完成 Ubuntu 系统安装映像文件的下载。

然后按上面创建虚拟机的过程创建一个命名为 Ubuntu 的虚拟电脑。注意以下选项：

- 电脑系统类型应该为 Linux。
- 电脑系统版本应该为 Ubuntu(64-bit)。
- 内存大小 2048MB。
- 硬盘大小 20GB。

作为参考，你可以对照下图核对一下，看看是否和我创建的一致。

![创建的 Ubuntu 虚拟机参数](https://www.zzxworld.com/resource/posts/202206/1654871955.7986.png)

确认一致后继续。

选择刚创建的虚拟电脑，点击右边的设置按钮，打开设置界面，并选择「存储」项：

![存储设置界面](https://www.zzxworld.com/resource/posts/202206/1654871979.9188.png)

选择「控制器：IDE」下的「没有盘片」，然后点击最右侧的蓝色光盘图标，在弹出的菜单中选择「选择虚拟盘」，然后选择刚刚下载的 Ubuntu 系统安装映像文件。

操作完成后的设置界面应该如下：

![选择 Ubuntu 安装映像文件后的设置界面](https://www.zzxworld.com/resource/posts/202206/1654872003.3066.png)

点击右下方的确定按钮回到主界面。然后点击右上方的启动按钮启动虚拟电脑。这时会新打开一个类似屏幕的窗口运行虚拟电脑，并显示 Ubuntu 系统的安装菜单。

![Ubuntu 系统安装菜单](https://www.zzxworld.com/resource/posts/202206/1654872025.7406.png)

接下来参照「[Ubuntu Linux 系统安装流程](https://www.zzxworld.com/posts/ubuntu-jammy-jellyfish-linux-install-guide)」这篇文章完成整个安装过程即可。这样就完成了一个 VirtualBox 虚拟机的创建，而且安装好了 Ubuntu Linux 系统。

## 安装增强功能

VirtualBox 默认安装好的桌面系统，在使用体验上并不是很好。比如鼠标操作，存在肉眼可见的迟滞感。以我虚拟的一台 Win10 为例，来看看如何改善这个问题。

![Win10 虚拟机窗口](https://www.zzxworld.com/resource/posts/202206/1654943636.3323.png)

上面是 Win10 虚拟机启动后的界面。鼠标右键点击窗口最下方的光盘图标，在弹出的菜单中选择「移除虚拟盘」。

![移除虚拟盘操作](https://www.zzxworld.com/resource/posts/202206/1654943685.5201.png)

然后选择窗口上方的「设备」菜单，选择最下面的「安装增强功能」菜单。稍等片刻，虚拟机中的系统应该会弹出这个来自光驱设备的自动运行提示。

![安装提示](https://www.zzxworld.com/resource/posts/202206/1654943709.2205.png)

选择「运行 VBoxWindowsAdditions.exe」，随后会弹出一个安全提示框，选择「是」继续。开始 VirtualBox Guest Additions 程序的安装。

![开始安装 VirtualBox Guest additions](https://www.zzxworld.com/resource/posts/202206/1654943862.5083.png)

安装过程很简单，只需要无脑点 Next 按钮就行了。完成后会提示需要重启系统。

![VirtualBox Guest Additions 安装完成提示](https://www.zzxworld.com/resource/posts/202206/1654943750.8723.png)

保持默认选择，点击 Finish 按钮，系统会开始重启。重新进入系统后，会发现鼠标的移动和操作顺滑了许多。

如果虚拟机是 Linux，在点击了「安装增强功能」菜单后，需要在命令端进行操作。相关命令按操作顺序分别如下：

```bash
# 创建挂载目录
sudo mkdir /mnt/cdrom

# 挂载光驱
sudo mount /dev/cdrom /mnt/cdrom

# 进入挂载的而光驱目录
cd /mnt/cdrom

# 安装增强功能
sudo ./VBoxLinuxAdditions.run
```

安装 VirtualBox 增强功能除了对鼠标操作有所改善，在虚拟显卡性能方面也有所增强。建议在虚拟任何附带桌面环境的操作系统后，都安装一下这个增强功能。

## 和主机共享文件

虚拟机在一些场景下免不了要和主机之间互相交换文件，对于这个需求，可以通过 VirtualBox 的共享文件夹功能来解决。

在虚拟机运行界面的下方，找到一个类似文件夹样式的图标，它是灰色的，表示当前没有共享任何目录，右键点击它。

![点击共享文件夹](https://www.zzxworld.com/resource/posts/202206/1654944029.5649.png)

选择弹出的「共享文件夹」菜单，进入共享文件夹设置界面。

![共享文件夹设置界面](https://www.zzxworld.com/resource/posts/202206/1654944051.472.png)

在这个界面的最右侧，有一个带加号的文件夹图标，点击它打开添加共享文件夹的操作界面。

![添加共享文件夹界面](https://www.zzxworld.com/resource/posts/202206/1654944074.0926.png)

这个界面有如下几个操作项：

- **共享文件夹路径**： 选择主机上要共享给虚拟机的目录。
- **共享文件夹名称**： 自定义这个共享的名称，方便自己识别即可。
- **只读分配**： 设置共享目录的操作权限，选择后共享目录在虚拟机环境中只可查看，不能做文件编辑操作。
- **自动挂载**： 选择后虚拟机点确定后，包括下次启动时，自动加载这个共享目录。
- **挂载点**: 指定共享目录在虚拟机中的位置。如果虚拟机是 Windows 系统，这里只能填盘符。
- **固定分配**： 选择后表示这个共享目录在当前虚拟机一直有效。不选就是临时有效。

在 Windows 虚拟机上挂载的共享目录以网络驱动器的方式实现。比如我挂在了一个命名为 test 的共享文件夹，在 Win10 虚拟机的文件资源管理器中看起来如下。

![Win10 挂载的共享文件夹](https://www.zzxworld.com/resource/posts/202206/1654944100.5291.png)

## 使用 USB 外设

除了使用文件共享的方式和虚拟机传输数据，VirtualBox 也支持虚拟机直连 USB 外设。使用这个功能前，需要先安装 VirtualBox 的扩展插件。插件下载地址：

[Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)*VirtualBox 官方网站的下载页面。*

打开这个页面后，找到 **VirtualBox Extension Pack** 这部分，下面有个 **All supported platforms** 的链接，点击即可下载到一个以 `.vbox-extpack` 后缀结尾的文件。

回到 VirtualBox 主界面，选择「管理」菜单，然后选择「全局设定」。在打开的全局设定窗口左侧，选择「扩展」项：

![全局设定扩展设置](https://www.zzxworld.com/resource/posts/202206/1654944593.97.png)

点击最右侧带加号的按钮，然后选择刚下载的扩展文件，并确认安装。安装好扩展后的界面如下：

![已安装扩展后的界面](https://www.zzxworld.com/resource/posts/202206/1654944615.2309.png)

点击确定返回 VirtualBox 主界面。接下来打开虚拟机的设置窗口。

![虚拟机设置窗口](https://www.zzxworld.com/resource/posts/202206/1654944194.3029.png)

控制器选择 USB3.0，然后点击筛选器右侧带加号的按钮，应该能看到当前系统上可用的 USB 外设。

![可用的 USB 外设](https://www.zzxworld.com/resource/posts/202206/1654944302.595.png)

选择要在虚拟机中直连的 USB 设备后，点击「确定」按钮返回主界面。启动虚拟机后，就能在虚拟机的系统中看到选择的 USB 设备了。

如果主机是 Linux 系统，并使用命令方式安装的 VirtualBox。还需要一些额外操作才能成功添加 USB 设备。

首先需要添加 `usbfs` 用户组。

```bash
sudo groupadd usbfs
```

然后把当前用户添加到 `usbfs` 和 `vboxusers` 用户组。

```bash
sudo usermod -aG usbfs $USER
sudo usermod -aG vboxusers $USER
```

重启一下主机系统，再按上面的步骤操作即可。

## 网络设置

在虚拟机的设置界面有一个网络设置项：

![虚拟机网络设置](https://www.zzxworld.com/resource/posts/202206/1654944442.6843.png)

VirtualBox 提供了多种网络类型，本文以应用为目的，所以不打算挨个介绍它们。仅以我的日常使用为参考，说说其中两种常用的网络模式：「网络地址转换(NAT)」和「桥接网卡」。

**网络地址转换(NAT)** 是 VirtualBox 创建虚拟机的默认网络模式。这是一种比较安全的单向网络访问模式。在虚拟机中可以访问主机所在的网络，包括互联网。但外部这些网络不能访问到虚拟机。如果必须要访问，只能通过配置端口转发的方式来实现。

**桥接网卡** 就比较粗暴了，虚拟机会占用主机网络的 IP。选择这种网络模式相当于把虚拟机放到了和主机平级的网络环境，暴露在主机网络下，所以和网络中任意主机都是互通状态。

对于这两种网络模式的使用选择，我会优先考虑网络地址转换模式，不会在主机网络暴露虚拟机，通过端口准发的方式也能达到桥接网卡模式的功能，但更可控。桥接网卡模式会是最后不得以的选择。

## 使用命令模式

除了使用界面来操作虚拟机，VirtualBox 还提供了更加高效的终端命令操作模式。比如创建虚拟机可以使用: `vboxmanage createvm` 命令，启动虚拟机可以使用: `vboxmanage startvm` 命令……。在界面上能对虚拟机进行的控制操作，在命令模式下基本都可以达到。

至于使用命令来操作虚拟机的好处，主要体现在效率和规范化方面。因为命令可以通过代码的方式来组织和编写，对于一些经常要重复创建和使用的虚拟机，直接通过代码把相关操作编写成脚本。之后使用的时候就只需要执行一下脚本，符合自己要求的虚拟机就创建好并启动起来了，是不是比自己用鼠标来一步步操作要更加有效率？

除此之外，使用命令模式还能实现界面端无法做到的功能。比如开机时自动在后台启动一个指定的虚拟机。

命令模式是对 VirtualBox 更加深度的应用领域，只有在一些专业领域才会用到。