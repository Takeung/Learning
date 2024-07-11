# Ubuntu22.04安装sougoupinyin

> 摘录自https://blog.csdn.net/qq_41209915/article/details/135518249

#### 1、更新应用源

```bash
sudo apt update
```

#### 2、安装输入法系统

```bash
sudo apt-get install fcitx
```

#### 3、打开系统设置

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/6a5147a4eca340bcb996a3c8363b770a.png)

#### 4、打开语言支持窗口

① 设置键盘输入法系统为: fcitx
② 添加或删除语言: 中文简体、英文
③ 应用到整个系统
④ 重启系统

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/f1bb77f0847347aa80ecad0fe399be05.png)

#### 5、设置fcitx开机自启动

```bash
# 将fcitx.desktop文件复制到开机自启动目录中
# 命令格式: sudo cp "fcitx.desktop文件所在的位置"  "开机自启动目录"
sudo cp /usr/share/applications/fcitx.desktop /etc/xdg/autostart/
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/b610f70c211544048c1d93cb0a2a937b.png)

#### 6、卸载ibus输入法系统

```bash
sudo apt purge ibus
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/74d8afce66c4431ab0a1d40660e4f758.png)

#### 7、下载搜狗输入法

[搜狗输入法官网下载地址](https://shurufa.sogou.com/)

需要选择适合你个人电脑的CPU架构进行下载，最好是直接使用Ubuntu系统里的Firefox浏览器下载, 下载后的文件名称大致如此: `sogoupinyin_4.2.1.145_amd64.deb`

#### 8、安装搜狗输入法

两种安装方式, 选择其一

> 方式一: 命令方式安装
>

```bash
# sudo dpkg -i "安装包所在路径"
sudo dpkg -i "/home/getter/Downloads/sogoupinyin_4.2.1.145_amd64.deb"
```

> 方式二: 可视化安装

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/873635ef367d4eb3894d1a655ac0d0ff.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/0ce3d5aa645c4037b0ddf830ac660a56.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/95adc36c97d848a684f4efe965bf76ac.png)

#### 9、安装搜狗输入法所需要的其它依赖工具

安装完成后重启

```bash
sudo apt install libqt5qml5 libqt5quick5 libqt5quickwidgets5 qml-module-qtquick2 libgsettings-qt1
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/4ca2962b095c4948b87b7f53329ca871.png)

#### 10、添加搜狗输入法到语言栏

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/22c3587cd81341ad8e0ded73f7a482dd.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/5de05620db3e485390a1473452dba756.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/894a7cd3943c41b19c27fcb1ba1d857a.png)

#### 11、测试使用输入法

打开一个文档或者记事本, 使用快捷键[Ctrl + 空格] 或 [Shift] 切换到搜狗输入法, 就可以使用啦!

补充: 如果按了快捷键似乎没有反应, 依旧是英文输入法, 那么可以尝试重启或者在右上角的输入法中手动切换一下

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/fde9b2f57e414a9cbfab43dac64d43a2.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/0efdfeeeb72d4cefa370dabf4da518d0.png)
