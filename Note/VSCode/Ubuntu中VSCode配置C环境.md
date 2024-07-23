# Ubuntu中VSCode配置C环境

- [Ubuntu中VSCode配置C环境](#ubuntu中vscode配置c环境)
  - [1 安装VSCode](#1-安装vscode)
  - [2 安装库](#2-安装库)
  - [3 安装插件](#3-安装插件)
  - [4 运行程序](#4-运行程序)
  - [5 调试程序](#5-调试程序)
    - [5.1 launch.json](#51-launchjson)
    - [5.2 tasks.json](#52-tasksjson)


## 1 安装VSCode

进入应用商店搜索visual studio code并点击install进行安装

安装后在终端输入code就能运行Vs Code

## 2 安装库

打开终端输入如下命令

```bash
sudo apt-get update
sudo apt-get install gcc
sudo apt-get install g++
sudo apt-get install gdb
```

结束后可以输入命令查看是否安装成功

```bash
gcc -v
gdb -v
g++ -v
```

## 3 安装插件

打开Vs Code搜索安装`C/C++`和`Code Runner`插件

## 4 运行程序

安装了Code Runner插件，就可以直接运行程序（但是还不能调试）：点击右上角的三角形按钮就能运行程序

![](https://s2.loli.net/2024/07/23/E7CbDHdYK4JocMU.png)

点击左下角的齿轮`Manage`，选择`settings`进入设置界面，命令行搜索`code runner run in terminal`并勾选

![](https://s2.loli.net/2024/07/23/HndVqwTXoyNSsJ6.png)

关闭`VSCode`再重新打开即可使用`Code Runner`运行程序了。

## 5 调试程序

只有`coderunner`无法进行调试，需要先在.vscode文件夹下新建名为launch.json和tasks.json的两个文件。

> 切换工作文件夹调试都要先重建.vscode子文件夹并配置

### 5.1 launch.json

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "C/C++",
            "type": "cppdbg",
            "request": "launch",
            "program": "${fileDirname}/${fileBasenameNoExtension}",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "preLaunchTask": "compile",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```

### 5.2 tasks.json

> 如果是c语言，需要将command项由g++改为gcc
>

```json
{
	"version": "2.0.0",
	"tasks": [
		{
			"type": "cppbuild",
			"label": "compile",
			"command": "/usr/bin/gcc",
			"args": [
				"-fdiagnostics-color=always",
				"-g",
				"${file}",
				"-o",
				"${fileDirname}/${fileBasenameNoExtension}"
			],
			"options": {
				"cwd": "${fileDirname}"
			},
			"problemMatcher": [
				"$gcc"
			],
			"group": "build",
			"detail": "compiler: /usr/bin/gcc"
		}
	]
}
```

保存完成之后，选择左侧的调试并点击左上角的绿色箭头，设置一个断点，即可进行调试测试。

> 需要确保tasks的label项与launch.json的preLaunchTask一致，否则调试时会报[“找不到任务”的错误](https://blog.csdn.net/qq_43663476/article/details/124903618)