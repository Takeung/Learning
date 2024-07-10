# Git——在Ubuntu 22.04系统上的安装及配置

#### 什么是 [Git](https://so.csdn.net/so/search?q=Git&spm=1001.2101.3001.7020)?

Git 是一个[分布式版本控制系统](https://so.csdn.net/so/search?q=分布式版本控制系统&spm=1001.2101.3001.7020)，旨在跟踪软件开发过程中源代码的变化。它使多个开发人员能够在一个项目上进行协作，从而确保结构化和有组织的开发过程。

本文讲述了如何在Ubuntu 22.04上[安装](https://so.csdn.net/so/search?q=安装&spm=1001.2101.3001.7020) Git，提供来一个创建项目文件夹并将更改推送到GitHub存储库的示例。

## 1、更新系统

打开终端并运行以下 apt 命令以更新系统

```shell
$ sudo apt update
$ sudo apt upgrade -y
```

![Update-Ubuntu-System](https://img-blog.csdnimg.cn/img_convert/8e946f399ab750644d40f2bc8a3951f2.png)

## 2、安装 Git

使用以下命令在 Ubuntu 22.04 上安装 Git

```shell
$ sudo apt install git
```

当系统提示确认安装时，按“Y”，将自动在您的系统上下载并安装 Git。

![Install Git on Ubuntu 22.04](https://img-blog.csdnimg.cn/img_convert/c82a7154c80513b237f402ebc8a748fd.png)

安装 git 及其依赖项后，通过运行确认 Git 版本。

```shell
$ git --version
git version 2.34.1
$
```

## 3、配置 Git

替换以下 git 命令中的“name”和“email”配置值。

```shell
$ git config --global user.name "Pradeep Kumar"
$ git config --global user.email "pradeepantil@gmail.com"
```

![Configure Git](https://img-blog.csdnimg.cn/img_convert/dd61efc69e22428710ed97a661130e4a.png)

## 4、创建文件夹并初始化 Git 存储库

现在，让我们为项目创建一个新文件夹并初始化一个 Git存储库。

```shell
$ mkdir project_code
$ cd project_code
$ git init
```

![Initialize-Git-Repository-Ubuntu-Linux](https://img-blog.csdnimg.cn/img_convert/6a6d936ef8b4d3758c67f0126608f8b5.png)

## 5、更改并提交

将一些文件添加到项目文件夹或修改现有文件

![Git-Status-Command-Output-Ubuntu-Linux](https://img-blog.csdnimg.cn/img_convert/c5dbb05a3d8edd2d7902b28cfdeb75ec.png)

进行更改后，即可将其提交到 Git 存储库：

```shell
$ git add .
$ git commit -m "First Commit"
```

此命令暂存所有更改，并提交描述更改的消息。

![Git-Commit-Changes-Ubuntu-Linux](https://img-blog.csdnimg.cn/img_convert/2b10223c67290dc4bab93413a4796f16.png)

## 6、创建 GitHub Repository

在github帐户上登录您的帐户，单击左上角的“New”标志，按照说明在 GitHub 上创建新 Repository，如下所示：

![Create GitHub Repository](https://img-blog.csdnimg.cn/img_convert/e8a5484c3660db593b376803c6d37e5c.png)

单击“New”后，我们将获得 Repository 的以下指令集。

![New-Github-Repository-Instructions](https://img-blog.csdnimg.cn/img_convert/c771499278358c4f9618f67846240f98.png)

接下来，从您的 Ubuntu 22.04 系统生成 ssh 密钥并将其上传到您的 Github 帐户。

从终端运行 ssh-keygen 命令。

```shell
$ ssh-keygen
```

![Create-sshkeys-github-repository](https://img-blog.csdnimg.cn/img_convert/6895dd512bbeaadcdc6933f0ba4fbde8.png)

运行下面的 cat 命令，查看公钥内容。

```shell
$ cat ~/.ssh/id_rsa.pub
```

将其粘贴到您的 github 帐户上进行身份验证，如下所示：

![Upload-GitHub-SSH-Keys](https://img-blog.csdnimg.cn/img_convert/267ae6efb85ea5ba0183241978fa62b3.png)

创建 GitHub 存储库 和 SSH 密钥后，按照页面上提供的说明将本地存储库连接到远程 GitHub 存储库。

```shell
$ git remote add origin git@github.com:pradeepantil/project_code.git
$ git remote -v
```

![Add -Remote-GitHub-Repository-Ubuntu-Linux](https://img-blog.csdnimg.cn/img_convert/8b50aafd3340404709c2dc95d661aaf9.png)

## 7、将更改推送到 GitHub 存储库

最后，将本地更改推送到 GitHub 存储库，运行“git push”命令。

```shell
$ git push -u origin master
```

![Push Changes to Github Repository](https://img-blog.csdnimg.cn/img_convert/73c48d615d6003b5596d2f6f1e4cbe07.png)

上面的输出显示，存储库中的本地更改被推送到远程 github 存储库。

验证远程 GitHub 存储库中的更改。

![Content-GitHub-Repository-After-Pushing-Changes](https://img-blog.csdnimg.cn/img_convert/60e88b1f1a570c29d2f05c87319ee3c0.png)