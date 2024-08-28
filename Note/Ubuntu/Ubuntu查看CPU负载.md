# Ubuntu查看CPU负载

- [Ubuntu查看CPU负载](#ubuntu查看cpu负载)
    - [1. 使用 `top` 命令](#1-使用-top-命令)
    - [2. 使用 `htop` 命令](#2-使用-htop-命令)
    - [3. 使用 `mpstat` 命令](#3-使用-mpstat-命令)
    - [4. 使用 `vmstat` 命令](#4-使用-vmstat-命令)
    - [5. 使用 `sar` 命令](#5-使用-sar-命令)
    - [6. 使用 `uptime` 命令](#6-使用-uptime-命令)
    - [7. 使用 `nload` 或 `iftop` 监控网络带宽](#7-使用-nload-或-iftop-监控网络带宽)
    - [8. 使用图形化工具](#8-使用图形化工具)


在 Ubuntu 中，有多种方式可以查看 CPU 负载。以下是几种常用的方法：

### 1. 使用 `top` 命令
`top` 命令是一个实时显示系统任务的工具，可以监控 CPU 负载、内存使用情况等。
```bash
top
```
在 `top` 输出中，您可以在屏幕的顶部看到 CPU 使用率，包含以下信息：
- `%Cpu(s)`: 各个 CPU 核心的使用率。以下是各个字段的含义：
  - `us`: 用户空间程序的 CPU 使用率
  - `sy`: 内核空间的 CPU 使用率
  - `ni`: 用户进程的优先级改变后的 CPU 使用率
  - `id`: 空闲 CPU 百分比
  - `wa`: 等待 I/O 操作的 CPU 百分比
  - `hi`: 硬件中断占用的 CPU 百分比
  - `si`: 软件中断占用的 CPU 百分比
  - `st`: 被虚拟机偷走的 CPU 时间

### 2. 使用 `htop` 命令
`htop` 是 `top` 命令的增强版，具有更友好的图形界面，可以通过箭头键上下滚动查看不同的进程。
```bash
sudo apt-get install htop
htop
```
`htop` 提供了更直观的 CPU 负载显示，包括每个核心的实时负载图。

### 3. 使用 `mpstat` 命令
`mpstat` 是 `sysstat` 工具包的一部分，可以详细显示每个 CPU 核心的使用情况。
```bash
sudo apt-get install sysstat
mpstat -P ALL
```
这会显示每个核心的使用情况，包括用户空间、系统空间、空闲时间等。

### 4. 使用 `vmstat` 命令
`vmstat` 命令提供了系统的整体状态，包括 CPU、内存、I/O 等信息。要查看 CPU 负载，可以运行：
```bash
vmstat 1
```
这会每秒刷新一次 CPU 和其他系统资源的状态。

### 5. 使用 `sar` 命令
`sar` 也是 `sysstat` 工具的一部分，可以按时间段记录 CPU 的使用情况。
```bash
sudo apt-get install sysstat
sar -u 1 3
```
这会每秒输出一次 CPU 使用情况，持续 3 次。

### 6. 使用 `uptime` 命令
`uptime` 命令显示系统的运行时间、登录用户数量以及平均负载。
```bash
uptime
```
输出中的平均负载指的是在 1 分钟、5 分钟、15 分钟内，系统运行队列的平均长度。通常低于 1.0 表示系统没有超负荷。

### 7. 使用 `nload` 或 `iftop` 监控网络带宽
如果怀疑 CPU 负载与网络相关，可以使用 `nload` 或 `iftop` 监控网络流量。

### 8. 使用图形化工具
在 Ubuntu 的 GUI 环境下，可以使用系统监视器 (System Monitor)，这是一个图形化工具，可以监控 CPU、内存、网络使用情况。
- 打开方式：`系统设置 -> 系统监视器`。

通过以上工具，您可以根据需要选择合适的方式来监控和分析 Ubuntu 系统的 CPU 负载。