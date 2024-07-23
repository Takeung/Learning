# IAR构建烧录全流程

- [IAR构建烧录全流程](#iar构建烧录全流程)
  - [1 打开工作空间](#1-打开工作空间)
  - [2 选择期望工程](#2-选择期望工程)
  - [3 设置输出格式](#3-设置输出格式)
  - [4 项目构建](#4-项目构建)
  - [5 脚本生成应用文件](#5-脚本生成应用文件)
  - [6 OTA升级](#6-ota升级)
    - [6.1 连接串口助手](#61-连接串口助手)
    - [6.2 SSH登录SOC远端](#62-ssh登录soc远端)
    - [6.3 拷贝应用文件到SOC远端](#63-拷贝应用文件到soc远端)
    - [6.4 执行烧录脚本](#64-执行烧录脚本)


## 1 打开工作空间

默认工作空间文件为`mcu_bsp\cpress-traveo_ii-bsp_freertos\tviibh8m\tools\iar\flash\`路径下的`tviibh8m_flash_cm0plus_cm7_0_cm7_1_template.eww`。

![](https://s2.loli.net/2024/07/23/QgOVMb5mtXYdp2W.png)

## 2 选择期望工程

选择某个项目之后，右键弹出菜单后选中`Set as Active`，设置为选中状态。

![](https://s2.loli.net/2024/07/23/GWVvJs7z6LAjeIE.png)

## 3 设置输出格式

同样在右键菜单中选择`Options...`选项，调整输出格式为`S-records`。

![](https://s2.loli.net/2024/07/23/QGY8prsT1LP9752.png)

## 4 项目构建

完成前述配置后，在右键菜单选择`Make`选项，等待完成项目构建。

![](https://s2.loli.net/2024/07/23/YBZf6VmPwhJ7iQX.png)

## 5 脚本生成应用文件

移动到`mcu_bsp\cpress-traveo_ii-bsp_freertos`路径下，执行`generateA_app_bin_CYT2B7.bat`脚本，自动生成三个核心下的可执行文件`cm0plus_app.bin`、`cm7_0_app.bin`以及`cm7_1_app.bin`。

![](https://s2.loli.net/2024/07/23/dRQx8EzeYw6XsWO.png)

## 6 OTA升级

### 6.1 连接串口助手

连接串口助手后，观测串口输出的日志。

OTA版本下，整机上电后MCU吐出日志仅有两行：

```c
*********************T24DG26 R1.0_V1.0.0.2.A_ORIN*************************
*********************T24DG26 R1.0_V1.0.0.2.aaaaaaaaaaaaa************************
```

> 在整机上电状态下，可能出现串口连接不识别的情况，此时可以把整机下电后再连接。

### 6.2 SSH登录SOC远端

以141整机为例，

```bash
ssh nvidia@172.26.1.141		# 密码：nvidia
```

### 6.3 拷贝应用文件到SOC远端

跳转到脚本所在路径，默认在`~/Downloads/BIN`路径下

![](https://s2.loli.net/2024/07/23/zBw7tbgJFsiSpa2.png)

根据项目和SOC版本不同，选择相对应的文件夹，此处以`B_ORIN`为例，

![](https://s2.loli.net/2024/07/23/EMHd7kvlU9n6yVN.png)

将需要升级的应用文件拷贝到该路径下

```bash
scp cm0plus_app.bin nvidia@172.26.1.141:~/Downloads/BIN/B_ORIN/
scp cm7_0_app.bin nvidia@172.26.1.141:~/Downloads/BIN/B_ORIN/
scp cm7_1_app.bin nvidia@172.26.1.141:~/Downloads/BIN/B_ORIN/
```

> 1、注意确认文件上传后的日期是否为最新；
>
> 2、如果目标SOC上不存在BIN路径或者缺少升级脚本，需要手动拷贝[BIN目录](https://qr.dingtalk.com/page/yunpan?route=previewDentry&spaceId=24505350245&fileId=147493068556&type=folder)到远端。

### 6.4 执行烧录脚本

保持在`~/Downloads/BIN/B_ORIN`路径下，

![image-20240723170854723](/home/tyjt/.config/Typora/typora-user-images/image-20240723170854723.png)

脚本中对应5行烧录指令，分别对应

- mcu 1：只有在烧录OTA功能时才需要；
- mcu 2、3：更新应用侧app部分；
- mcu 4：切换OTA分区，执行完会使设备重启；
- mcu 5：退出OTA模式，执行完会使设备重启

整机刷写策略是：先刷任意分区之后，切换分区将另一分区刷写。

实际操作时，可以拷贝上述指令逐条执行，**同时监测SOC串口日志**，确保刷写成功；
或者修改脚本，单区烧录成功后自行切换分区。

> 脚本开发者补充：如若升级cm0，也要一起烧录cm7_0和cm7_1。