# Ubuntu 22.04日常办公环境记录

- [Ubuntu 22.04日常办公环境记录](#ubuntu-2204日常办公环境记录)
  - [Git：万物之基](#git万物之基)
  - [Terminator：程序员必用终端](#terminator程序员必用终端)
  - [VSCode：强大](#vscode强大)
    - [1. 下载 VSCode `.deb` 包](#1-下载-vscode-deb-包)
    - [2. 安装 `.deb` 包](#2-安装-deb-包)
    - [3. 插件](#3-插件)
  - [Chrome：核心](#chrome核心)
    - [1. 下载 Google Chrome `.deb` 包](#1-下载-google-chrome-deb-包)
    - [2. 安装 `.deb` 包](#2-安装-deb-包-1)
  - [Flameshot：截图](#flameshot截图)
    - [1. 更新软件包列表](#1-更新软件包列表)
    - [2. 安装 Flameshot](#2-安装-flameshot)
  - [Meld：对比工具](#meld对比工具)
  - [Typora：码字神器](#typora码字神器)
  - [PicGo：图床](#picgo图床)
    - [1. 下载 PicGo 的 AppImage](#1-下载-picgo-的-appimage)
    - [2. 给 AppImage 文件添加执行权限](#2-给-appimage-文件添加执行权限)
    - [3. 运行 PicGo](#3-运行-picgo)
    - [4. 使用 `picgo` 命令启动](#4-使用-picgo-命令启动)
    - [5. 配置 PicGo](#5-配置-picgo)
  - [Okular：PDF阅读](#okularpdf阅读)
    - [1. 更新系统](#1-更新系统)
    - [2. 安装 Okular](#2-安装-okular)
    - [3. 启动 Okular](#3-启动-okular)
  - [VirtualBox：Ubuntu办公必备](#virtualboxubuntu办公必备)
    - [1. 更新系统](#1-更新系统-1)
    - [2. 添加 VirtualBox 官方仓库](#2-添加-virtualbox-官方仓库)
    - [3. 添加 Oracle 的公钥](#3-添加-oracle-的公钥)
    - [4. 安装 VirtualBox](#4-安装-virtualbox)
    - [5. （可选）安装 VirtualBox Extension Pack](#5-可选安装-virtualbox-extension-pack)
    - [6. 启动 VirtualBox](#6-启动-virtualbox)
    - [7. （可选）安装 VirtualBox Guest Additions](#7-可选安装-virtualbox-guest-additions)
  - [sougoupinyin：Ubuntu办公必备](#sougoupinyinubuntu办公必备)
  - [SunloginClient：Ubuntu办公有备无患](#sunloginclientubuntu办公有备无患)
    - [1. 下载 SunloginClient](#1-下载-sunloginclient)
    - [2. 安装必要的依赖](#2-安装必要的依赖)
    - [3. 安装 SunloginClient](#3-安装-sunloginclient)
    - [4. 启动 SunloginClient](#4-启动-sunloginclient)
    - [5. 配置 SunloginClient](#5-配置-sunloginclient)
    - [6. 更新和卸载](#6-更新和卸载)
  - [CuteCom：串口调试](#cutecom串口调试)
    - [1. 更新系统](#1-更新系统-2)
    - [2. 安装 `cutecom`](#2-安装-cutecom)
    - [3. 启动 `cutecom`](#3-启动-cutecom)
    - [4. 配置 `cutecom`](#4-配置-cutecom)
    - [5. 常见问题](#5-常见问题)
    - [总结](#总结)


## [Git：万物之基](https://github.com/Takeung/Learning/blob/dev/Note/Git/Git%E2%80%94%E2%80%94%E5%9C%A8Ubuntu%2022.04%E7%B3%BB%E7%BB%9F%E4%B8%8B%E5%AE%89%E8%A3%85%E5%8F%8A%E9%85%8D%E7%BD%AE.md)

## Terminator：程序员必用终端

安装命令：

```bash
sudo apt-get install terminator
```

## VSCode：强大

### 1. 下载 VSCode `.deb` 包

- 访问 [Visual Studio Code 的下载页面](https://code.visualstudio.com/Download)。
- 下载适用于 Debian 和 Ubuntu 的 `.deb` 包（通常是 `*.deb` 文件）。

### 2. 安装 `.deb` 包

打开终端，使用以下命令安装下载的包：

```bash
sudo dpkg -i path_to_your_file/code_*.deb
sudo apt-get install -f  # 用于解决可能的依赖问题
```
替换 `path_to_your_file` 为 `.deb` 文件的实际路径。

### 3. 插件

VsCode插件无奇不有，常用的是Python和C++插件。

**[Python插件](https://fishros.com/d2lros2/#/humble/chapt1/basic/5.玩转Ubuntu之常用软件?id=python插件)**

![image-20210723125628815](https://fishros.com/d2lros2/humble/chapt1/basic/5.%E7%8E%A9%E8%BD%ACUbuntu%E4%B9%8B%E5%B8%B8%E7%94%A8%E8%BD%AF%E4%BB%B6/imgs/image-20210723125628815.png)

**C++**

![image-20210909005135905](https://fishros.com/d2lros2/humble/chapt1/basic/5.%E7%8E%A9%E8%BD%ACUbuntu%E4%B9%8B%E5%B8%B8%E7%94%A8%E8%BD%AF%E4%BB%B6/imgs/image-20210909005135905.png)

CodeRunner

![](https://s2.loli.net/2024/07/23/3ICvuHcNUpoXQ1L.png)

**[汉化插件](https://fishros.com/d2lros2/#/humble/chapt1/basic/5.玩转Ubuntu之常用软件?id=汉化插件)**

![image-20210720135816630](https://fishros.com/d2lros2/humble/chapt1/basic/5.%E7%8E%A9%E8%BD%ACUbuntu%E4%B9%8B%E5%B8%B8%E7%94%A8%E8%BD%AF%E4%BB%B6/imgs/image-20210720135816630.png)

**听歌网易抑云**

![image-20210720113510268](https://fishros.com/d2lros2/humble/chapt1/basic/5.%E7%8E%A9%E8%BD%ACUbuntu%E4%B9%8B%E5%B8%B8%E7%94%A8%E8%BD%AF%E4%BB%B6/imgs/image-20210720113510268.png)

**背单词**

![image-20210720135841441](https://fishros.com/d2lros2/humble/chapt1/basic/5.%E7%8E%A9%E8%BD%ACUbuntu%E4%B9%8B%E5%B8%B8%E7%94%A8%E8%BD%AF%E4%BB%B6/imgs/image-20210720135841441.png)

**[看小说](https://fishros.com/d2lros2/#/humble/chapt1/basic/5.玩转Ubuntu之常用软件?id=看小说)**

![image-20210720135941635](https://fishros.com/d2lros2/humble/chapt1/basic/5.%E7%8E%A9%E8%BD%ACUbuntu%E4%B9%8B%E5%B8%B8%E7%94%A8%E8%BD%AF%E4%BB%B6/imgs/image-20210720135941635.png)

**[股票](https://fishros.com/d2lros2/#/humble/chapt1/basic/5.玩转Ubuntu之常用软件?id=股票)**

![image-20210720140207445](https://fishros.com/d2lros2/humble/chapt1/basic/5.%E7%8E%A9%E8%BD%ACUbuntu%E4%B9%8B%E5%B8%B8%E7%94%A8%E8%BD%AF%E4%BB%B6/imgs/image-20210720140207445.png)

**CMake插件**

![](https://s2.loli.net/2024/07/18/xDNpcEF3RQIwdmo.png)

**vscode-icons**

![](https://s2.loli.net/2024/07/18/6BxLhbeSz75AyTi.png)

**ROS插件**

![image-20240718180640939](/home/tyjt/.config/Typora/typora-user-images/image-20240718180640939.png)

**Msg Language Support**

![](https://s2.loli.net/2024/07/18/BigZ3cxtzPENV4F.png)

**IntelliCode**

![](https://s2.loli.net/2024/07/18/Q6xWbEFPG3NZvgY.png)

**URDF**

![](https://s2.loli.net/2024/07/18/WHUnv7sPLgtpx2u.png)

**Markdown All in One**

![](https://s2.loli.net/2024/07/18/I6ksuSXzTKcp3NA.png)

## Chrome：核心

### 1. 下载 Google Chrome `.deb` 包

- 访问 [Google Chrome 的下载页面](https://www.google.com/chrome/)。
- 点击“下载 Chrome”按钮，然后选择 `.deb` 包进行下载。

### 2. 安装 `.deb` 包

打开终端并使用以下命令安装下载的包：

```bash
sudo dpkg -i ~/Downloads/google-chrome-stable_current_amd64.deb
sudo apt-get install -f
```
这里的路径 `~/Downloads/google-chrome-stable_current_amd64.deb` 是你下载的 `.deb` 文件的路径。如果你将其下载到其他位置，请相应地修改路径。

## Flameshot：截图

### 1. 更新软件包列表

打开终端并执行：

```bash
sudo apt update
```

### 2. 安装 Flameshot

```bash
sudo apt install flameshot
```

Flameshot依赖Qt5的一些库，初次配置时可能需要额外注意补充。

## Meld：对比工具

Meld是一个跨平台地、可视化地、对比及合并工具。它提供文件和目录的双向和三向比较，并支持许多流行的版本控制系统。

Meld特点：

- 文件和目录的双向和三向比较。
- 文件比较随着你的输入而更新。
- 自动合并模式和对变更块的操作有助于使合并更容易。
- 可视化让你更容易比较你的文件。
- 支持`Git, Bazaar, Mercurial, Subversion`等。

输入如下命令即可安装：

```
sudo apt-get install meld
```

Meld很简单、易用且强大，大家可以自行去体验体验。

![img](https://static.mianbaoban-assets.eet-china.com/xinyu-images/MBXY-CR-6c5bc0f8357d4a47ab538bdeb82795b5.png)

![img](https://static.mianbaoban-assets.eet-china.com/xinyu-images/MBXY-CR-100ef5a0e4808b62625fb9c02fff46d1.png)

除此之外，meld还能与git等版本控制系统结合使用。

## [Typora：码字神器](https://github.com/Takeung/Learning/blob/dev/Note/Linux/Ubuntu22.04%E6%BF%80%E6%B4%BBTypora.md)

## PicGo：图床

### 1. 下载 PicGo 的 AppImage

PicGo 提供了一个 AppImage 格式的可执行文件，这使得在 Linux 系统上运行变得简单。

1. **访问 PicGo 的 GitHub 发布页面**：
   前往 [PicGo GitHub Releases 页面](https://github.com/Molunerfinn/PicGo/releases) 下载最新的 AppImage 文件。

2. **下载 AppImage 文件**：
   找到最新版本的 `PicGo-xxx.AppImage` 文件，点击下载。

3. **移动到合适的目录**：
   可以将下载的文件移动到你的 `~/Downloads` 目录或其他你想要的目录。

### 2. 给 AppImage 文件添加执行权限

在终端中，使用以下命令来给予 AppImage 文件执行权限：

```bash
cd ~/Downloads  # 如果你将文件下载到 Downloads 目录
chmod +x PicGo-2.4.0-beta.7.AppImage  # 替换为实际的文件名
```

### 3. 运行 PicGo

使用以下命令启动 PicGo：

```bash
./PicGo-2.4.0-beta.7.AppImage  # 替换为实际的文件名
```

### 4. 使用 `picgo` 命令启动

如果你提到的 `picgo` 是在 `PATH` 中的一个命令或脚本，确保它指向你的 `AppImage` 文件，可以使用以下方法：

**创建符号链接**：

将 `PicGo-2.4.0-beta.7.AppImage` 文件的符号链接创建到 `/usr/local/bin`（或任何在 `PATH` 中的目录），然后可以使用 `picgo` 命令来启动：

```bash
sudo ln -s /path/to/PicGo-2.4.0-beta.7.AppImage /usr/local/bin/picgo
```

> 注意：`/path/to/PicGo-2.4.0-beta.7.AppImage`必须是根目录下的绝对路径！

然后，直接在终端中输入 `picgo` 启动应用：

```bash
picgo
```

### 5. 配置 PicGo

启动 PicGo 后，你可以通过其图形界面进行配置，设置图床服务（需要额外注册SMMS账号并[获取API Token](https://smms.app/home/apitoken)）、上传图片等。

## Okular：PDF阅读

在 Ubuntu 22.04 上安装 Okular，可以通过以下步骤完成。Okular 是一个功能强大的文档查看器，支持多种文件格式，如 PDF、EPUB、DJVU 等。

### 1. 更新系统

首先，确保你的系统是最新的：

```bash
sudo apt update
sudo apt upgrade
```

### 2. 安装 Okular

Okular 是 KDE 桌面环境的文档查看器，可以通过以下命令安装：

```bash
sudo apt install okular
```

### 3. 启动 Okular

安装完成后，你可以通过以下命令启动 Okular：

```bash
okular
```

或者在应用菜单中找到 Okular 图标并点击启动。

## [VirtualBox：Ubuntu办公必备](https://github.com/Takeung/Learning/blob/dev/Note/VirtualBox/VirtualBox%E5%85%A5%E9%97%A8%E4%BD%BF%E7%94%A8%E4%B8%8E%E8%BF%9B%E9%98%B6%E9%85%8D%E7%BD%AE.md)

VirtualBox 是一个广泛使用的开源虚拟机管理程序，用于在主机操作系统上运行虚拟机。

### 1. 更新系统

首先，确保你的系统是最新的：

```bash
sudo apt update
sudo apt upgrade
```

### 2. 添加 VirtualBox 官方仓库

为了确保你获得 VirtualBox 的最新版本，我们需要添加 Oracle 的官方仓库：

```bash
sudo apt install software-properties-common
```

添加 VirtualBox 仓库：

```bash
sudo add-apt-repository "deb [arch=amd64] https://download.virtualbox.org/virtualbox/debian focal contrib"
```

### 3. 添加 Oracle 的公钥

添加 VirtualBox 仓库的公钥，以确保下载的软件包的真实性：

```bash
wget -q https://www.virtualbox.org/download/oracle_vbox_2016.asc -O- | sudo apt-key add -
```

### 4. 安装 VirtualBox

更新包列表并安装 VirtualBox：

```bash
sudo apt update
sudo apt install virtualbox-7.0
```

### 5. （可选）安装 VirtualBox Extension Pack

VirtualBox Extension Pack 提供了额外的功能，如 USB 2.0/3.0 支持、RDP、磁盘加密等。可以从 [VirtualBox 官网](https://www.virtualbox.org/wiki/Downloads) 下载 Extension Pack 的最新版本。

下载完成后，使用以下命令安装：

```bash
sudo VBoxManage extpack install /path/to/extension-pack.vbox-extpack
```

替换 `/path/to/extension-pack.vbox-extpack` 为下载的 Extension Pack 的路径。

### 6. 启动 VirtualBox

安装完成后，你可以通过以下命令启动 VirtualBox：

```bash
virtualbox
```

或者在应用菜单中找到 VirtualBox 图标并点击启动。

### 7. （可选）安装 VirtualBox Guest Additions

如果你在虚拟机中运行 Ubuntu 或其他操作系统，可以安装 Guest Additions 以获得更好的性能和功能支持。在虚拟机内打开终端，运行以下命令：

```bash
sudo apt update
sudo apt install virtualbox-guest-dkms virtualbox-guest-utils virtualbox-guest-x11
```

通过以上步骤，你应该能够在 Ubuntu 22.04 上成功安装并运行 VirtualBox。

## [sougoupinyin：Ubuntu办公必备](https://github.com/Takeung/Learning/blob/dev/Note/Linux/Ubuntu22.04%E5%AE%89%E8%A3%85sougoupinyin.md)

## SunloginClient：Ubuntu办公有备无患

### 1. 下载 SunloginClient

首先，前往 [SunloginClient 的官方网站](https://sunlogin.oray.com/) 下载适用于 Linux 的客户端。通常，下载的文件会是一个 `.deb` 包文件。

你可以使用浏览器访问上述链接，并下载最新版本的 `.deb` 文件。

### 2. 安装必要的依赖

在安装 `SunloginClient` 之前，你需要确保系统中安装了所有必要的依赖。

```bash
sudo apt update
sudo apt install -f
```

> 可能会缺少依赖库`libgconf-2-4`，通过以下命令安装
>
> ```bash
> sudo apt install libgconf-2-4
> ```

### 3. 安装 SunloginClient

在下载完 `.deb` 文件后，你可以使用以下命令进行安装。假设你已经下载了文件，并且它位于你的 `Downloads` 文件夹中。

```bash
cd ~/Downloads
sudo dpkg -i sunloginclient*.deb
```

如果在安装过程中遇到依赖问题，可以使用以下命令来修复：

```bash
sudo apt-get install -f
```

> 或者使用以下命令自动修复依赖问题
>
> ```bash
> sudo apt --fix-broken install
> ```

### 4. 启动 SunloginClient

安装完成后，你可以通过应用菜单找到并启动 SunloginClient，也可以在终端中使用以下命令启动：

```bash
sunloginclient
```

### 5. 配置 SunloginClient

首次启动 SunloginClient 时，你需要进行配置。按照屏幕上的提示完成配置步骤，包括登录你的 Sunlogin 账户以及设置远程控制的相关参数。

### 6. 更新和卸载

- **更新**: 如果需要更新 SunloginClient，可以下载最新版本的 `.deb` 文件，然后重复上述安装步骤。
- **卸载**: 要卸载 SunloginClient，可以使用以下命令：

  ```bash
  sudo apt-get remove --purge sunloginclient
  ```

  如果还希望删除所有相关的配置文件，可以使用 `--purge` 选项。

## CuteCom：串口调试

`cutecom` 是一个图形化的串口终端程序，适用于串口通信。以下是安装 `cutecom` 的步骤：

### 1. 更新系统

首先，确保你的系统是最新的。打开终端并运行：

```bash
sudo apt update
sudo apt upgrade
```

### 2. 安装 `cutecom`

使用以下命令来安装 `cutecom`：

```bash
sudo apt install cutecom
```

### 3. 启动 `cutecom`

安装完成后，你可以通过应用菜单找到 `cutecom` 并启动它，或者在终端中输入：

```bash
cutecom
```

### 4. 配置 `cutecom`

启动 `cutecom` 后，你需要进行串口配置。你可以选择连接到特定的串口设备，设置波特率、数据位、停止位等参数来进行串口通信。

### 5. 常见问题

如果你遇到权限问题（例如无法访问串口设备），可以将你的用户添加到 `dialout` 组：

```bash
sudo usermod -a -G dialout $USER
```

然后重新启动你的计算机或注销并重新登录，以使权限更改生效。

### 总结

1. 更新系统
2. 安装 `cutecom`
3. 启动 `cutecom` 并配置串口设置
4. 确保用户具有串口访问权限
