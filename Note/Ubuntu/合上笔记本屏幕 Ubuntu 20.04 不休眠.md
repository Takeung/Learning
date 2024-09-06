# 合上笔记本屏幕 Ubuntu 20.04 不休眠

将系统配置文件/etc/systemd/logind.conf  的其中一行，改成如下所示：

```shell
sudo vim /etc/systemd/logind.conf
```

```shell
HandleLidSwitch=ignore
```

重启服务：

```shell
sudo service systemd-logind restart
```

或者 

```shell
sudo systemctl restart systemd-logind
```

可以设置的参数：

```shell
ignore
poweroff
reboot
halt
kexec
suspend
hibernate
hybrid-sleep
suspend-then-hibernate(
lock
```

`HandlePowerKey`是指按下电源键

`HandleLidSwitch`是指笔记本屏幕合起来，设置为`ignore`，忽略，也就是啥都不改变，依然如故；

永远不休眠：

```shell
sudo systemctl mask sleep.target suspend.target hibernation.target hybrid-sleep.target
```

恢复休眠：

```shell
sudo systemctl unmask sleep.target suspend.target hibernation.target hybrid-sleep.target
```