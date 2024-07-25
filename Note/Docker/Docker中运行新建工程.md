# Docker中运行新建工程

1 查看已安装的docker镜像列表

```bash
docker ps -a
```

![](https://s2.loli.net/2024/07/25/e8NkT52GpdKQBMh.png)

> 可能需要sudo权限

2 拷贝工程目录

```bash
docker cp example_project ros2_humble:/workspace/
```

![](https://s2.loli.net/2024/07/25/En7D8lIRHOYXZBT.png)

3 运行docker镜像

```bash
docker exec -it ros2_humble bash
```

进入到传入的工程目录

4 编译构建

```bash
colcon build
```

5 运行ROS2通信