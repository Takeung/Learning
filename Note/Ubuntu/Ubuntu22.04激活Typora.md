## Ubuntu22.04激活Typora

## 1激活环境准备

### 1.1安装Python3、Python3-pip

#### 步骤 1：更新系统

在安装软件之前，建议先更新系统的软件包索引：

```bash
sudo apt update
```

#### 步骤 2：安装 Python 3

Ubuntu 22.04 默认已经安装了 Python 3，但是如果你需要安装特定版本，可以使用以下命令：

```bash
sudo apt install python3
```

安装完成后，你可以通过以下命令检查 Python 版本：

```bash
python3 --version
```

#### 步骤 3：安装 Python 3 的包管理器 `pip`

`pip` 是 Python 的包管理工具，用于安装和管理 Python 包。要安装 `pip`，可以使用以下命令：

```bash
sudo apt install python3-pip
```

安装完成后，你可以通过以下命令检查 `pip` 版本：

```bash
pip3 --version
```

#### 步骤 4：（可选）安装 Python 3 的开发工具和库

如果你打算进行 Python 开发，可能还需要安装一些开发工具和库，例如 Python 的头文件和编译工具：

```bash
sudo apt install python3-dev python3-venv build-essential
```

### 1.2安装nodejs

```bash
sudo apt-get install nodejs
```

### 1.3克隆typoraCraker项目

```bash
git clone https://github.com/cnvetman/typoracracker.git
```

### 1.4切换到克隆下来的项目根目录typoracracker下执行

切换到克隆下来的项目根目录typoracracker下执行：

```bash
pip3 install -r requirements.txt
```

## 2解包替换文件

以下操作都是切换到typoraCracker项目根目录下执行，可以解包原生文件到app_asar中

### 2.1下载Typora1.0.3版本

项目目录下有typora_1.0.3的deb包，执行下面命令直接安装即可。

```bash
dpkg -i typora_1.0.3_amd64.deb
```

### 2.2解包原生app.asar

安装Typora后，原生app.asar文件默认存储路径是/usr/share/typora/resources/app.asar；

可通过以下命令解包原生app.asar文件：

```bash
python3 typora.py /usr/share/typora/resources/app.asar app_asar/  
```

解包后，在`app_asar`路径下会有一个`dec_app`目录。

### 2.3修改License.js

修改`app_asar/dec_app`目录中的License.js；在typoraCracker项目下，提供有修改好的License.js，所以直接替换即可：

```bash
cp example/patch/License.js app_asar/dec_app/
```

### 2.4生成app.asar

```bash
python3 typora.py -u app_asar/dec_app app_asar
```

在app_asar路径下，会生成新的的app.asar文件

### 2.5替换app.asar

将Typora原生的的app.asar文件替换：

备份原生app.asar文件

```bash
sudo cp /usr/share/typora/resources/app.asar /usr/share/typora/resources/app.asar.bak    
```

用新生成的app.asar文件替换typora自带的app.asar文件

```bash
sudo cp app_asar/app.asar /usr/share/typora/resources/app.asar
```

## 3激活Typora

在typoraCracker项目根路径下，执行keygen.js脚本：

生成激活码

```bash
node example/keygen.js
```

得到激活码后，打开**Typora软件** --> Typora帮助 --> **我的许可证** --> 输入你的激活信息，随便一个邮箱加生成的激活码。

激活成功

![](https://s2.loli.net/2024/07/05/5bDKTosA1XcOGfv.png)