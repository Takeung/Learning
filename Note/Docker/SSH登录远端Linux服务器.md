# SSH登录远端Linux服务器

## 1 VSCode登录远端主机

1.1 首先选择远程连接按钮

![](https://s2.loli.net/2024/07/17/F1VTysLxeIA8mi6.png)

控制台会弹出可选服务，其中就有SSH登录选项（如果尚未安装插件，会提示安装）

点击登录后，会提示输入完整的SSH登录命令

![](https://s2.loli.net/2024/07/17/AUlQTJHbgmPwXNq.png)

> 举例：`ssh nvidia@172.26.1.95`

而后输入密码登录即可。

1.2 登录成功之后，选择打开文件夹，默认会弹出可用目录供以选择：

![](https://s2.loli.net/2024/07/17/oJwDIikr2dCAhNf.png)

> 此处，实际用到的目录为`/home/nvidia/git/jetson-ros2-workspace/`

## 2 命令行登录远端运行docker

如果是通过命令行登录远端呢？

同样是先用SSH顺利登录，之后通过以下命令获取可运行的docker镜像

```bash
docker ps -a
```

![](https://s2.loli.net/2024/07/17/aUygKJfBDmcRxX2.png)

然后直接运行镜像即可

```bash
docker exec -it ros2-foxy bash
```

![](https://s2.loli.net/2024/07/17/mdxOS9goKfBYcuX.png)

启动运行后，通过测试ros2命令验证通过。

> docker中工作空间为`/workspace/ros2_ws`，与上文中远端主机上的实际工作空间互为映射。