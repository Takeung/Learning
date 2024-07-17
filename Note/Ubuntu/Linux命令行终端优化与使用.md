# Linux命令行终端优化与使用

[TOC]

> 摘自：https://cloud.tencent.com/developer/article/1729659

## 0 简述前言

描述：Shell的类型有很多种并且本身自带的特性，但是对于用户来说远远不能满足用户的DIY，所以 Shell 配置框架孕育而生，常用的Shell配置框架如下：

**bash 配置框架是 Bash-it**

- PS：linux下shell默认的是bash使用是相当广泛的；虽然bash的功能已经很强大，但对于以懒惰为美德的程序员来说，bash的提示功能不够强大，界面也不够炫并非理想工具，但是我们可以通过使用其扩展框架bash-it。 Github：[https：//github.com/Bash-it/bash-it](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2FBash-it%2Fbash-it&source=article&objectId=1729659)

**zsh 配置框架是 Oh My Zsh**

- PS：zsh的功能极其强大，只是配置过于复杂，起初只有极客才在用，后经大佬开发创建了一个名为oh-my-zsh的开源项目（通用性强：Ubuntu Win10 均可用； 自此，只需要简单的安装配置，小白程序员们都可以用上高档大气上档次，狂拽炫酷吊炸天的`oh my zsh` 官方网站：[https：//ohmyz.sh/](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fohmyz.sh%2F&source=article&objectId=1729659) Github：[https：//github.com/robbyrussell/oh-my-zsh](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2Frobbyrussell%2Foh-my-zsh&source=article&objectId=1729659)

------

## 1 Bash终端美化

描述：Bash-it 配置框架从社区收集了许多实用的命令和脚本，主要包括别名、自动补全代码、定制函数、以及提示符主题等四大类型。

### 1.1 环境安装

（1）安装 Bash-it 之前，首先需要确认的是系统中是否含有 git 命令

```
$ which git
/usr/bin/git
```

如果没有采用操作系统的软件包管理器进行下载

```
yum install -y git
```

（2）使用 git 命令将 Bash-it 克隆到用户主目录下的 .bash_it 子目录：

```
git clone --depth=1 https://github.com/Bash-it/bash-it.git ~/.bash_it
```

（3）执行 install.sh 安装脚本来安装 Bash-it

```
cd !$
./install.sh -s
```

install.sh 脚本包括下列 3 个选项

```
# 1.--interactive (-i)：这个选项允许我们交互式选择要启用哪些别名、自动补全和插件。
# 2.--silent (-s)：静默安装，没有任何输入提示。
# 3.--no-modify-config (-n)：不修改现有的 bash 配置文件 .bashrc 或.bash_profile。

您想要保留您的.bashrc并在末尾追加bash-it模板吗?[y / N] N
原有的 .bashrc 配置文件将备份为 .bashrc.bak
-rw-r--r--   1 root root 1.9K May  9 14：30 .bashrc
-rw-r--r--   1 root root 3.2K May  8 16：44 .bashrc.bak
```

（4）配置生效

```
source ~/.bashrc
# 2020-05-09 14：33：51 ⌚  vm-1575613390 in ~
# ○ →
```

![WeiyiGeek.安装成功](https://ask.qcloudimg.com/http-save/yehe-1389665/luik9str6r.png)

### 1.2 bash-it 命令

基础语法：

```javascript
bash-it ： Bash-it help and maintenance
parameters：
    1： verb [one of： help | show | enable | disable | migrate | update | search | version | reload ]
    2： component type [one of： alias(es) | completion(s) | plugin(s) ] or search term(s)
    3： specific component [optional]
examples：
    $ bash-it show plugins
    $ bash-it help aliases
    $ bash-it enable plugin git [tmux]...
    $ bash-it disable alias hg [tmux]...
    $ bash-it migrate
    $ bash-it update  #更新 Bash-it
    $ bash-it search [-|@]term1 [-|@]term2 ... [ -e/--enable ] [ -d/--disable ] [ -r/--refresh ] [ -c/--no-color ]
    $ bash-it version
    $ bash-it reload
```

### 1.3 aliases（别名）

基础示例：

（1）查看启用的别名

```javascript
bash-it show aliases | less
#第一列为别名的名称，第二列显示该别名是否启用（启用的别名在 [] 中有 X），最后一列是有关别名的说明
Alias               Enabled?  Description
ag                    [ ] (未启用)    the silver searcher (ag) aliases
```

（2）启动或者禁用别名

```
$ bash-it enable alias  <alias name> [alias name]... -or- $ bash-it enable alias all
$ bash-it disable alias <alias name> [alias name]... -or- $ bash-it disable alias all
bash-it enable alias git
git enabled with priority 150.
```

（3）查看到底有那些git 别名

```
bash-it help aliases git | less
# gca='git commit -v -a'
# gcm='git commit -v -m'
# gcam="git commit -v -am'
# gci='git commit --interactive'
# gcamd='git commit --amend'
# gb='git branch'
# gba='git branch -a'
# gbt='git branch --track'
# gbm='git branch -m'
# gbd='git branch -d'
# gbD='git branch -D'
# gcount='git shortlog -sn'
# gcp='git cherry-pick'
# gcpx='git cherry-pick -x'
# gco='git checkout'
# gcom='git checkout master'
# gcb='git checkout -b'
# gcob='git checkout -b'
# gct='git checkout --track'
# gcpd='git checkout master; git pull; git branch -D'
# gexport='git archive --format zip --output'
# gdel='git branch -D'
# gmu='git fetch origin -v; git fetch upstream -v; git merge upstream/master'
# gll='git log --graph --pretty=oneline --abbrev-commit'
# gg="git log --graph --pretty=format：'%C(bold)%h%Creset%C(magenta)%d%Creset %s %C(yellow)<%an> %C(cyan)(%cr)%Creset' --abbrev-commit --date=relative'
# ggs="gg --stat'
# gsl="git shortlog -sn'
# gwc="git whatchanged'
# gt="git tag'
# gta="git tag -a'
# gtd="git tag -d'
# gtl="git tag -l'
# gpatch="git format-patch -1'
# gnew="git log [email protected]{1}[email protected]{0}'
# gcaa="git commit -a --amend -C HEAD'
# gprom="git fetch origin master && git rebase origin/master && git update-ref refs/heads/master origin/master'
# gpunch="git push --force-with-lease'
# ggui="git gui'
# gcsam="git commit -S -am'
# gst="git stash'
# gstb="git stash branch'
# gstd="git stash drop'
# gstl="git stash list'
# gstpu="git stash push'
# gstpum="git stash push -m'
# gsts="git stash push'
# gstsm="git stash push -m'
# gstpo="git stash pop'
# gstp="git stash pop'
# gsw="git switch'
# gswm="git switch master'
# gswc="git switch --create'
# gswt="git switch --track'
# ghm='cd "$(git rev-parse --show-toplevel)'
# gh='ghm'
# gu='git ls-files . --exclude-standard --others'
# gtls="git tag -l | gsort -V'
# gtls='git tag -l | sort -V'

bash-it disable alias gitsvn
# gitsvn disabled
```

### 1.4 completions（补全）

基础示例：

（1）查看启用了哪些自动补全

```javascript
bash-it show completions | less
# Completion          Enabled?  Description
# apm                   [ ]
# awless                [ ]
# awscli                [ ]
# bash-it               [x]
```

（2）启动或者禁用自动补全(使用方式同上)

```
$ bash-it enable completion  <completion name> [completion name]... -or- $ bash-it enable completion all
$ bash-it disable completion <completion name> [completion name]... -or- $ bash-it disable completion all
```

### 1.5 plugins（插件）

基础示例：

（1）查看启用了哪些插件

```javascript
bash-it show plugins | less
Plugin              Enabled?  Description
alias-completion      [x]     Automatic completion of aliases
```

（2）启动或者禁用插件(使用方式同上)

```
$ bash-it enable plugin  <plugin name> [plugin name]... -or- $ bash-it enable plugin all
$ bash-it disable plugin <plugin name> [plugin name]... -or- $ bash-it disable plugin all
```

### 1.6 search（搜索内容）

描述：Bash-it 还提供了一个非常快捷的方式来查找所需的内容，比如我们想要看看有关 tmux 和 ansible 的情况;

基础示例：

**示例1** 查看插件别名以及补全()

```javascript
○ → bash-it search base
      plugins：  base
 2020-05-09 15：50：36 ⌚  vm-1575613390 in ~
○ → bash-it search git
      aliases：  git gitsvn
      plugins：  autojump git git-subrepo jgitflow jump
  completions：  git git_flow git_flow_avh
```

**示例2** 除了通过 bash-it enable 命令来启用别名、自动补全和插件外，我们也可以在搜索模块和组件时加以启用

```
~$ bash-it search git --enable
# aliases： git gitsvn
# plugins： autojump fasd git git-subrepo jgitflow jump  #本例中的gitsvn、jgitflow、git_flow 也一并启用了
# completions： git git_flow git_flow_avh
```

*注意事项：*

- 1.如果出现执行 `bash-it search base` 出现以下错误

  - #问题 

    ```
    bash： alias： /usr/bin/egrep： not found 
    -bash： alias： -E： not found -bash： alias：    \[： not found  
    ```

    #解决方法

    ```
    unalise egrep
    ```

    2.可以通过下列命令来分别启用所有的别名、自动补全和插件

  ```
  bash−it enable alias all
  bash-it enable completion all
  ```

### 1.7 Theme主题更改

描述：Bash-it 随附了大约 50 多个提示符主题样式，如果想要看看这些主题的真实外观，那么我们可以执行下面的命令：

```javascript
# 1.查看已存在的主题外观样式
$ BASH_PREVIEW=true bash-it reload
主题名称：zork
┌─[root][vm-1575613390][±][master ✓][~/.bash_it]
└─▪
┌─[root][vm-1575613390][±][master ✓][~/.bash_it]
└─▪ exit
```

下面列举个人喜欢的主题：

```
- powerline-multiline
- powerline
- modern-time
- mairan
- bira
- agnoster
```

#2.更改主题我们需要直接编辑 .bashrc配置文件

```
vim ~/.bashrc
# Lock and Load a custom theme file.
# Leave empty to disable theming.
# location /.bash_it/themes/
# 将单引号中的内容（bobby）替换成别的主题名称（如 zork），并保存即可。
export BASH_IT_THEME='zork'
```

#3.为了使新设置的提示符主题生效，你需要关闭并重新打开终端，或者注销并重新登录

```
source !$
Ctrl+D
```

![WeiyiGeek.zork](https://ask.qcloudimg.com/http-save/yehe-1389665/35ghgkw7a2.png)

为了使新设置的提示符主题生效，你需要关闭并重新打开终端，或者注销并重 新登录。

### 1.8 自定义定制

描述：Bash-it 的确为我们提供了不少好用的别名、自动补全和插件,我们可以对其进行定制的机制,可以定制的内容包括`别名、自动补全、插件、主题样式`等等

路径和名称如下：

```javascript
• aliases/custom.aliases.bash：别名
• completion/custom.completion.bash：自动补全
• lib/custom.bash：库
• plugins/custom.plugins.bash：插件
• themes/<theme name>/<theme name>.theme.bash：主题样式
```

在此，我们以如何定制别名为例来说明，其它类型的定制方法类似，无非就是以特定的名称命名并放在确定的目录。 

**Step1.**首先，我们在 aliases 目录下使用文本编辑器（如 vim）创建 custom.aliases.bash文件

```javascript
~$ cd ~/.bash_it/aliases/available
~$ vim custom.aliases.bash
```

**Step2.**接着添加具体的别名内容

```javascript
#!/bin/bash
cite 'about-alias'
about-alias 'Custom aliases for convenience.'

alias sd='shutdown -h now'
alias up='uptime'
```

**Step3.**利用以下命令来查看并启动

```javascript
$ bash-it show aliase
custom                [ ]     Custom aliases for convenience.

$ bash-it enable alias custom
custom is already enabled.
```

**Step4.**再重新加载一下配置以及效果查看

```javascript
$ bash-it reload

┌─[root][vm-1575613390][±][maste][~/.bash_it/aliases/available]
└─▪ up
 16：23：58 up 131 days, 17：55,  1 user,  load average： 0.18, 0.09, 0.06
```

------

## 2 Zsh终端美化

环境依赖：zsh(肯定需要这个啦)，一些front字体安装; 

### 2.1 安装流程

查看系统当前的shell类型

```javascript
$ echo $SHELL 
/bin/bash
```

查看系统是否安装了zsh

```
$ cat /etc/shells 
/bin/sh
/bin/bash
/sbin/nologin
/usr/bin/sh
/usr/bin/bash
/usr/sbin/nologin
/bin/tcsh
/bin/csh
```

安装zsh

```
$ sudo apt-get install zsh
$ yum -y install zsh
$ brew install zsh # mac安装
```

将zsh设置为默认shell

```
chsh -s /bin/zsh # CentOS
```

Mac如下在 /etc/shells 文件中加入如下一行

```
/usr/local/bin/zsh  # 接着运行下面命令
chsh -s /usr/local/bin/zsh
```

安装oh-my-zsh源码是放在github上，先确保你的机器上已安装了git

1.自动安装

```javascript
wget https：//github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | sh #成功会出现oh-my-zsh界面结果
```

2.手动安装

```
git clone git：//github.com/robbyrussell/oh-my-zsh.git ~/.oh-my-zsh
cp ~/.oh-my-zsh/templates/zshrc.zsh-template ~/.zshrc
```

### 2.2 zsh目录说明

```javascript
~/.oh-my-zsh master  ls
cache               CONTRIBUTING.md  lib          log           plugins(插件)    templates(主题模板)  tools
CODE_OF_CONDUCT.md  custom(自定义模板)           LICENSE.txt  oh-my-zsh.sh  README.md  themes
```

### 2.3 zsh主题修改 

oh-my-zsh配置文件：`~/.zshrc`

```javascript
$ls ~/.zshrc
/home/ubuntu/.zshrc
```

默认主题是robbyrussell（更多主题参考来源） 

```
sed -r -i.bak 's#ZSH_THEME="[a-z].+"#ZSH_THEME="agnoster"#' ~/.zshrc
```

更新配置：

```
$ source ~/.zshrc
```

附录主题：

- [agnoster-fcamblor主题] ： `git clone https：//github.com/fcamblor/oh-my-zsh-agnoster-fcamblor.git`

### 2.4 插件安装

 (1) 自动补齐插件

将[此插件]()放到oh-my-zsh目录的插件库下：

```javascript
$ mkdir ~/.oh-my-zsh/plugins/incr 
$ wget http：//mimosa-pudica.net/src/incr-0.2.zsh -P ~/.oh-my-zsh/plugins/incr/

#在~/.zshrc文件末尾加上
echo 'source ~/.oh-my-zsh/plugins/incr/incr*.zsh' >> ~/.zshrc
```

(2) 语法高亮插件

注意：将目录切换到~/.oh-my-zsh/custom/plugins

```javascript
#克隆插件到自定义插件目录之中
git clone https：//github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM：-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

#Activate the plugin in ~/.zshrc：
plugins=(git zsh-syntax-highlighting)
```

(3) 字体设置

进入终端设置（WSL的话，点击窗口左上角，然后属性）,其他lINUX有发行界面也同样，修改字体为FiraCode Retina。

```javascript
https：//raw.githubusercontent.com/tonsky/FiraCode/master/distr/ttf/FiraCode-Retina.ttf  #安装字体 FiraCode （浏览器可以不进中英文切换）
https：//github.com/powerline/powerline/raw/develop/font/PowerlineSymbols.otf # （浏览器需要进行中英文切换）
https：//github.com/abertsch/Menlo-for-Powerline.git #优先建议使用(宽字节可使用)
```

**Refenrence**

- zsh主题：[https：//github.com/robbyrussell/oh-my-zsh/wiki/themes](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2Frobbyrussell%2Foh-my-zsh%2Fwiki%2Fthemes&source=article&objectId=1729659)
- zsh插件： [https：//github.com/robbyrussell/oh-my-zsh/wiki/Plugins](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2Frobbyrussell%2Foh-my-zsh%2Fwiki%2FPlugins&source=article&objectId=1729659)

------

### 2.5 WSL优化配置

*什么是WSL？*

Windows Subsystem for Linux（简称WSL）是一个为在Windows 10上能够原生运行Linux二进制可执行文件（ELF格式）的兼容层，Windows10里可以用Linux的终端了。

启动启用WSL（Ubuntu）, 参看Windows10常用配置.md

WSL安装后的目录位置：`C：\Users\用户名\AppData\Local\Packages\CanonicalGroupLimited.UbuntuonWindows_79rhkp1fndgsc\LocalState\rootfs`

*WSL初步配色调料包：*

```javascript
字体：consolas
字体大小：24
屏幕文字颜色：220 220 220
屏幕背景颜色：50 50 50
不透明度：95%
```

WSL采用Solarized配色调料包：

```javascript
背景rgb（0，43，53）
文字rgb(147,161,161)
```

WSL的主题配色临时工具：ColorTool

可能未来的新终端里就会集成主题功能了这个工具貌似是可以使用那些iTerm2上的漂亮主题

```javascript
github源码：https：//github.com/Microsoft/Terminal/tree/master/src/tools/ColorTool
github下载：https：//github.com/microsoft/terminal/releases/tag/1904.29002
#后解压缩得到一个ColorTool.exe以及一个schemes主题文件夹
#比如我想要solarized_dark主题，然后使用WSL终端在目录里执行（PowerShell也行）
./ColorTool.exe -d schemes/solarized_dark.itermcolors
```

终端中显示：Wrote selected scheme to the defaults. 表示已经成功。重启WSL即可。

**配置流程步骤**

1.设置agnoster.zsh-theme的显示样式，不显示用户和主机名称：

```javascript
echo 'DEFAULT_USER="ubuntu"' >> ~/.zshrc  #改成当前登录用户名字即可
```

2.由于CMD的原因默认自带的蓝色的色调不能很好的看清楚，需要将其中blue修改为075，这样颜色会更容易辨认了

#操作1.将原本的主题文件复制一份更改（为了以后方便升级）

```javascript
cp ~/.oh-my-zsh/themes/agnoster.zsh-theme ~/.oh-my-zsh/custom/themes/agnoster_wsl.zsh-theme
```

#操作2.修改blue颜色 

```
# Dir： current working directory 201 line
prompt_dir() {
  #将blue改成075
  prompt_segment 075 $CURRENT_FG '%~' #gnoster_wsl.zsh
}    

# Dir： current working directory 
prompt_dir() {  
   prompt_segment 075 black '%~'  #agnoster-fcamblor
} 
```

#操作3.由于我们修改了主题名称，还需要去刚才的.zshrc配置文件修改主题为agnoster_wsl。这样，重启WSL就大功告成了。

```
sed -r -i.bak 's#ZSH_THEME="[a-z].+"#ZSH_THEME="agnoster"#' ~/.zshrc
```

系统支持的颜色表：

![WeiyiGeek.颜色表](https://ask.qcloudimg.com/http-save/yehe-1389665/gx8f3e5xmx.png)

最后效果如下：

![WeiyiGeek.效果示例](https://ask.qcloudimg.com/http-save/yehe-1389665/hzi29scbkk.png)

### 2.6 CloudStudioIDE终端配置

**Step1.**需要进行系统字体的环境设置

```javascript
sudo apt-get install fonts-powerline
# clone
git clone https：//github.com/powerline/fonts.git --depth=1
# install
cd fonts
./install.sh
```

**Step2.**下载指定的字体以及依赖，Liunx字体设置由于我们是从网页上显示，最好在客户端也进行安装字体

```javascript
wget https：//github.com/powerline/powerline/raw/develop/font/PowerlineSymbols.otf
mv PowerlineSymbols.otf ~/.local/share/fonts/
fc-cache -vf ~/.local/share/fonts/  #更新字体缓存
```

**Step3.**本地客户端安装PowerlineSymbol字体，并且在浏览器里面进行设置

![WeiyiGeek.浏览器字体设置](https://ask.qcloudimg.com/http-save/yehe-1389665/gtlrviw36t.png)

最后效果如下：

![WeiyiGeek.cloudStudio](https://ask.qcloudimg.com/http-save/yehe-1389665/iyj1as6xva.png)

*参考：*

- [https：//github.com/powerline/powerline](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2Fpowerline%2Fpowerline&source=article&objectId=1729659)
- [https：//github.com/powerline/fonts](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2Fpowerline%2Ffonts&source=article&objectId=1729659)
- [https：//powerline.readthedocs.io/en/latest/installation/linux.html#fonts-installation](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fpowerline.readthedocs.io%2Fen%2Flatest%2Finstallation%2Flinux.html%23fonts-installation&source=article&objectId=1729659)
- 字体下载
  - [FiraCode](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2Ftonsky%2FFiraCode&source=article&objectId=1729659)
  - [Menlo-for-Powerline](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2Fabertsch%2FMenlo-for-Powerline.git&source=article&objectId=1729659)
  - [PowerlineSymbols](https：//cloud.tencent.com/developer/tools/blog-entry?target=https%3A%2F%2Fgithub.com%2Fpowerline%2Fpowerline%2Fraw%2Fdevelop%2Ffont%2FPowerlineSymbols.otf&source=article&objectId=1729659)

------

### 2.7 VisualStudioCode

描述：为了让VScode终端正确的显示zsh的主题agnoster，在我们本地的VisualStudioCode进行选择我们安装的字体，让终端更能方便的显示;

```javascript
#Ubuntu系统进行安装字体
$cd /usr/share/fonts/truetype/
$sudo git clone https：//github.com/abertsch/Menlo-for-Powerline.git
$sudo fc-cache -f -v

# WeiyiGeek
# fc-cache -vf
# /usr/share/fonts： caching, new cache contents： 5 fonts, 2 dirs
# /usr/share/fonts/opentype： caching, new cache contents： 1 fonts, 0 dirs
# /usr/share/fonts/truetype： caching, new cache contents： 0 fonts, 1 dirs
# /usr/share/fonts/truetype/dejavu： caching, new cache contents： 6 fonts, 0 dirs
# /usr/local/share/fonts： caching, new cache contents： 0 fonts, 0 dirs
# /root/.local/share/fonts： skipping, no such directory
# /root/.fonts： skipping, no such directory
# /var/cache/fontconfig： cleaning cache directory
# /root/.cache/fontconfig： not cleaning non-existent cache directory
# /root/.fontconfig： not cleaning non-existent cache directory
fc-cache： succeeded
```

其他设置：

```javascript
#vscode进行设置终端字体
"editor.fontFamily"： "Consolas, 'Courier New', monospace",
"editor.fontLigatures"： false,
"terminal.integrated.fontFamily"： "Menlo for Powerline",
```

最终效果：

![WeiyiGeek.Vscode-oh-my-zsh](https://ask.qcloudimg.com/http-save/yehe-1389665/mh316eratd.png)