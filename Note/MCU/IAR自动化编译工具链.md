# IAR自动化编译工具链

- [IAR自动化编译工具链](#iar自动化编译工具链)
    - [0 前言](#0-前言)
      - [方案一：调用IarBuild.exe](#方案一调用iarbuildexe)
      - [方案二：调用编译器和链接器](#方案二调用编译器和链接器)
    - [1 使用编译工具链](#1-使用编译工具链)
  - [2 关键问题记录](#2-关键问题记录)
    - [2.1 config识别错误](#21-config识别错误)
    - [2.2 varfile未添加](#22-varfile未添加)
    - [2.3 IAR同时编译输出HEX和BIN文件](#23-iar同时编译输出hex和bin文件)
  - [3 补充材料](#3-补充材料)


### 0 前言

共有两种命令调用方案

#### 方案一：调用IarBuild.exe

已经完成IAR工程创建并且能在工具中编译成功

```shell
iarbuild project.ewp [ -clean | -build | -make | -cstat_analyze |  -cstat_clean] config[,config1,config2,...]|*[-log  errors|warnings|info|all][-parallel number][-varfile filename]
```

| Parameter         | Description                                                  |
| ----------------- | ------------------------------------------------------------ |
| project.ewp       | Your IAR Embedded Workbench project file.                    |
| -clean            | Removes any intermediate and output files.                   |
| -build            | Rebuilds and relinks all files in the specified build configuration(s). |
| -make             | Brings the specified build configuration(s) up to date by compiling,  assembling, and linking only the files that have changed since the last  build. |
| -cstat_analyze    | Analyzes the project using C-STAT and generates information about  the number of messages. For more information, see the C-STAT®  Static Analysis Guide. |
| -cstat_clean      | Cleans the C-STAT message database for the project. For more  information, see the C-STAT® Static Analysis Guide. |
| config\|*         | config, the name of a configuration you want to build, which can  be either one of the predefined configurations Debug or Release, or  a name that you define yourself. For more information about build  configurations, see Projects and build configurations, page 96. * (wild card character), the -clean, -build, and -make commands will a process all configurations defined in the project. |
| -log errors       | Displays build error messages.                               |
| -log warnings     | Displays build warning and error messages.                   |
| -log info         | Displays build warning and error messages, and messages issued by  the #pragma message preprocessor directive. |
| -log all          | Displays all messages generated from the build, for example compiler  sign-on information and the full command line. |
| -parallel number  | Specifies the number of parallel processes to run the compiler in to  make better use of the cores in the CPU. |
| -varfile filename | Makes custom-defined argument variables become defined in a  workspace scope available to the build engine by specifying the file to  use. See Configure Custom Argument Variables dialog box, page 88. |

#### 方案二：调用编译器和链接器

不依赖于既有IAR工程（.ewp），自由度更高，可以根据`BuildLog.log`依次执行编译、链接，最终完成工程的构建。

### 1 使用编译工具链

- 文本替换部分依赖于sed库，使用方法简介如下


> ```shell
> sed -i 's/{ecm_db_port}/'"${DATABASE_PORT}"'/g' "${CANAL_ADAPTER_CONF_DIR}/${WORKFLOW_CONF_ADAPTER_FILE}"
> ```
>
> 这行代码是使用 `sed`（stream editor）命令来对文件进行编辑的一个例子。`sed` 是一个非常强大的文本处理工具，常用于对文件进行查找、替换、删除等操作。下面是对这行代码的详细解释：
>
> 1. `sed`：这是命令本身，表示要使用流编辑器。
> 2. `-i`：这是 `sed` 的一个选项，表示对文件进行就地编辑，即直接修改原文件而不是输出到标准输出。
> 3. `'`：开始标志，用于定义 `sed` 的命令模式。
> 4. `s`：这是 `sed` 的替换命令，用于查找匹配的文本并替换为新的文本。
> 5. `/{ecm_db_port}/`：这是要被替换的模式，在这个例子中，它看起来像是一个占位符。
> 6. `"${DATABASE_PORT}"`：这是替换模式的值，`${DATABASE_PORT}` 是一个环境变量，其值将被插入到文件中替换 `{ecm_db_port}`。
> 7. `'`：结束标志，用于结束 `sed` 的命令模式。
> 8. `g`：全局标志，表示替换所有匹配的实例，而不是只替换第一个。
> 9. `"${CANAL_ADAPTER_CONF_DIR}"`：这是文件的路径，`${CANAL_ADAPTER_CONF_DIR}` 是一个环境变量，表示配置文件的目录。
> 10. `/`：路径分隔符，用于分隔目录和文件名。
> 11. `"${WORKFLOW_CONF_ADAPTER_FILE}"`：这是要编辑的文件名，`${WORKFLOW_CONF_ADAPTER_FILE}` 是一个环境变量，表示具体的配置文件。
>
> 将这行代码翻译成更易于理解的步骤：
>
> - 首先，找到环境变量 `CANAL_ADAPTER_CONF_DIR` 指定的目录和 `WORKFLOW_CONF_ADAPTER_FILE` 指定的文件。
> - 然后，使用 `sed` 命令打开这个文件。
> - 接着，查找文件中所有 `{ecm_db_port}` 的实例。
> - 最后，将每个 `{ecm_db_port}` 替换为环境变量 `DATABASE_PORT` 的值，并保存更改。
>

但windows下没有原生支持的sed，需要基于Git Bash添加环境变量后才可在CMD中直接使用，配置方法如下：

> #### [win10下用sed（使用cmd命令窗口直接使用）](https://www.cnblogs.com/lyongyong/p/13361807.html)
>
> 1、安装`git bash`，因为`git bash`自带`sed`
>
> ​    下载安装git （windows版）
>
> ​    网址：https://git-scm.com/download/win
>
> ​    点击for windows版本->下载
>
> 2、把sed添加到环境变量中`C:\Program Files\Git\usr\bin`
>
> 3、重新打开命令窗口，输入`sed`就能直接使用了

- **【不再需要】**文件格式`Srec-Bin`转换工具`srec_cat.exe`需要另外下载配置，目前直接放在`IarBuild.exe`同一路径下。
- 自动化编译工具依赖于正确输出BIN文件，而调试烧录需要使用HEX格式的应用文件，所以为了兼顾两方需求而避免频繁修改IAR工程配置，所以增加编译后处理程序，能够将正确编译得到的输出文件转换为所需的两种格式，具体操作流程参见2.3小节。

编写自动化编译批处理脚本如下：

```shell
@echo off
setlocal enabledelayedexpansion

set startTime=%time%

REM IAR编译和Srec-Bin转换工具
set IarBuild="C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.4\common\bin\IarBuild.exe"

REM 项目工程和依赖文件路径
set Cm0Ewp=%CD%\tviibh8m\tools\iar\flash\cm0plus_cm7_0_cm7_1\cm0plus.ewp
set Cm70Ewp=%CD%\tviibh8m\tools\iar\flash\cm7_0_mc\cm7_0.ewp
set Cm71Ewp=%CD%\tviibh8m\tools\iar\flash\cm7_1_mc\cm7_1.ewp
set Cm0VarFile=%CD%\tviibh8m\tools\iar\flash\tviibh8m_flash_cm0plus_cm7_0_cm7_1_template.custom_argvars
set Cm70VarFile=%CD%\tviibh8m\tools\iar\flash\tviibh8m_flash_cm7_0_mc_template.custom_argvars
set Cm71VarFile=%CD%\tviibh8m\tools\iar\flash\tviibh8m_flash_cm7_1_mc_template.custom_argvars
set src_cm0plus_Path=%CD%\tviibh8m\tools\iar\flash\cm0plus_cm7_0_cm7_1\rev_c\Exe\
set src_cm7_0_Path=%CD%\tviibh8m\tools\iar\flash\cm7_0_mc\rev_c\Exe\
set src_cm7_1_Path=%CD%\tviibh8m\tools\iar\flash\cm7_1_mc\rev_c\Exe\

REM A、B分区代码差异文件
set Cm0FilePath=%CD%\tviibh8m\src\app\main_cm0plus.c
set Cm70FilePath=%CD%\tviibh8m\src\app\hardware\Timer\user_timer.c

REM 输出目录
set OutputOrinA=Y:\MCU_BSP\BIN_Dev\A_ORIN
set OutputOrinB=Y:\MCU_BSP\BIN_Dev\B_ORIN
set OutputOtaUploader=Y:\MCU_BSP\BIN_Dev

set foundA=0
set foundB=0

REM 检查文件内容
for /f "usebackq delims=" %%a in ("%Cm0FilePath%") do (
    set line=%%a
    if "!line!"=="#define CURRENT_PARTITION 'A'" (
        set foundA=1
    ) else if "!line!"=="#define CURRENT_PARTITION 'B'" (
        set foundB=1
    )
)

if %foundA%==1 (
    sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm0FilePath%
    sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm70FilePath%
    @echo Compile the A-partition code using the IAR toolchain
) else if %foundB%==1 (
    sed -i "s/#define CURRENT_PARTITION 'A'/#define CURRENT_PARTITION 'B'/" %Cm0FilePath%
    sed -i "s/#define CURRENT_PARTITION 'A'/#define CURRENT_PARTITION 'B'/" %Cm70FilePath%
    @echo Compile the B-partition code using the IAR toolchain
) else (
    echo No matching definitions found
    exit /b
)

%IarBuild% %Cm0Ewp% -clean rev_c
%IarBuild% %Cm0Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm0VarFile%
%IarBuild% %Cm70Ewp% -clean rev_c
%IarBuild% %Cm70Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm70VarFile%
%IarBuild% %Cm71Ewp% -clean rev_c
%IarBuild% %Cm71Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm71VarFile%

if not exist %OutputOrinA%\ (
    mkdir %OutputOrinA%\
)
if not exist %OutputOrinB%\ (
    mkdir %OutputOrinB%\
)

if %foundA%==1 (
    copy %src_cm0plus_Path%\cm0plus_app.bin %OutputOrinA%\cm0plus_app.bin
    copy %src_cm7_0_Path%\cm7_0_app.bin %OutputOrinA%\cm7_0_app.bin
    copy %src_cm7_1_Path%\cm7_1_app.bin %OutputOrinA%\cm7_1_app.bin
    @echo.
    echo Switch to the B partition code
    sed -i "s/#define CURRENT_PARTITION 'A'/#define CURRENT_PARTITION 'B'/" %Cm0FilePath%
    sed -i "s/#define CURRENT_PARTITION 'A'/#define CURRENT_PARTITION 'B'/" %Cm70FilePath%
    @echo.
    @echo Compile the B-partition code using the IAR toolchain
) else if %foundB%==1 (
    copy %src_cm0plus_Path%\cm0plus_app.bin %OutputOrinB%\cm0plus_app.bin
    copy %src_cm7_0_Path%\cm7_0_app.bin %OutputOrinB%\cm7_0_app.bin
    copy %src_cm7_1_Path%\cm7_1_app.bin %OutputOrinB%\cm7_1_app.bin
    @echo.
    echo Switch to the A partition code
    sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm0FilePath%
    sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm70FilePath%
    @echo.
    @echo Compile the A-partition code using the IAR toolchain
) else (
    echo No matching definitions found
    exit /b
)

@REM %IarBuild% %Cm0Ewp% -clean rev_c
%IarBuild% %Cm0Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm0VarFile%
@REM %IarBuild% %Cm70Ewp% -clean rev_c
%IarBuild% %Cm70Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm70VarFile%
@REM %IarBuild% %Cm71Ewp% -clean rev_c
%IarBuild% %Cm71Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm71VarFile%

if %foundA%==1 (
    copy %src_cm0plus_Path%\cm0plus_app.bin %OutputOrinB%\cm0plus_app.bin
    copy %src_cm7_0_Path%\cm7_0_app.bin %OutputOrinB%\cm7_0_app.bin
    copy %src_cm7_1_Path%\cm7_1_app.bin %OutputOrinB%\cm7_1_app.bin
) else if %foundB%==1 (
    copy %src_cm0plus_Path%\cm0plus_app.bin %OutputOrinA%\cm0plus_app.bin
    copy %src_cm7_0_Path%\cm7_0_app.bin %OutputOrinA%\cm7_0_app.bin
    copy %src_cm7_1_Path%\cm7_1_app.bin %OutputOrinA%\cm7_1_app.bin
) else (
    echo No matching definitions found
    exit /b
)

@echo.
echo Switch back to the A partition code
sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm0FilePath%
sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm70FilePath%

REM 删除文件
del /F /Q %CD%\cm0plus_app.bin %CD%\cm7_0_app.bin %CD%\cm7_1_app.bin

@echo.
if %errorlevel% equ 0 (
    @echo The full-link compilation is successful
) else (
    @echo The full-link compilation fails, check the path and permissions
)

@echo.
@echo Start  time: %startTime%
@echo Finish time: %time%
@echo.

endlocal

pause
```

## 2 关键问题记录

### 2.1 config识别错误

按照搜索到的例子得到配置值多为Debug或者Release，但实际上对应到本项目工程中，实际应该为rev_c，对应IAR工具配置为：

```
Project -> Edit Configurations...
```

进而选择需要使用的配置

![](https://s2.loli.net/2024/09/04/yYfbhKtpurSsDcQ.png)

其他核心编译时使用的config参数同理。

### 2.2 [varfile未添加](https://e2e.ti.com/support/wireless-connectivity/zigbee-thread-group/zigbee-and-thread/f/zigbee-thread-forum/605771/rtos-cc2630-iar-command-line-build-failure-variable-expansion-failed-for-pre-build-command-line)

一开始未识别到varfile为必需的参数，执行时编译完成但是无法完成链接动作，相应的可执行文件自然无法生成。有关用到的varfile可以查看工作配置找到相应的存储位置：

```
Tools -> Configure Custom Argument Variables...
```

能够观察到相应的varfile存储在`flash`路径下，进入该路径后即可找到所需文件

![](https://s2.loli.net/2024/09/04/n4PydGRlJEvMLoW.png)

> 参考：[[IAR\] 编译报错:Variable expansion failed for Pre-Build command line](https://www.cnblogs.com/xuejiangqiang/p/17516544.html)

### 2.3 [IAR同时编译输出HEX和BIN文件](https://blog.csdn.net/Alenfun/article/details/134788977)

- Options->Build Actions -> Post-build command line 输入：

```shell
$PROJ_DIR$\post-build.bat "$TARGET_BPATH$"
```

![](https://s2.loli.net/2024/09/09/r8UY2lukHqIVBNb.png)

- 将post-build.bat文件放在工程目录下, post-build.bat内容如下：

```shell
set OUT=%1.out
set HEX=%1.hex
@REM 保持命名一致性
set BIN=%1_app.bin

:: calculate application checksum
ielftool --fill 0xFF;0x0-0xfffb --checksum __checksum:2,crc16,0x0;0x0-0xfffb --verbose %OUT% %OUT%

:: generate additional output: hex
ielftool.exe --ihex --verbose %OUT% %HEX%

:: generate additional output: binary
ielftool.exe --bin --verbose %OUT% %BIN%
```

- 编译工程，输出文件即有HEX和BIN文件

> 注：本项目中涉及到的三个工程都需要做此配置。

## 3 补充材料

> ### 降低Build时间
>
> #### 使能Parallel Build
>
> 使能Parallel Build (Tools > Options > Project > Enable parallel build) (注意：IDE 9版本以上，Enable parallel build是默认勾选的)：
>
> ![img](https://www.iar.com/siteassets/china/china-20230821-1.png)
>
> #### 正确包含头文件
>
> 确保头文件里面使用头文件保护，以避免头文件被重复包含：
>
> ```cpp
> #ifndef HFILE_H
> #define HFILE_H
>  /* ... */
> #endif
> ```
>
> 确保源文件里面只包含对应需要的头文件。源文件里面包含不需要的头文件可能不会影响编译的结果，但是会影响build时间（build的时候会对所有的头文件进行预处理），另外还会影响代码的可读性（一般添加对应的头文件说明跟对应模块有关联）。
>
> 可以通过查看对应的预处理输出来查看对应的头文件包含信息：Options > C/C++ Compiler > Preprocessor > Preprocessor output to file：
>
> ![img](https://www.iar.com/siteassets/china/china-20230821-2.png)
>
> 编译会输出对应的.i文件，里面会包含对应头文件包含信息：
>
> <img src="https://www.iar.com/siteassets/china/china-20230821-3.png" alt="img" style="zoom:50%;" />
>
> #### 创建和使用Library
>
> 对于一些不需要每次Re-build的代码（典型的有RTOS的内核静态代码，BSP的静态代码等），可以创建Library (Options > General Options > Output > Library)，然后在工程里面添加对应的Library (Options > Linker > Library > Additional libraries: (one per line))，这样build的时候对应Library的代码就不需要重新编译。
>
> ![img](https://www.iar.com/siteassets/china/china-20230821-4.png)
>
> ![img](https://www.iar.com/siteassets/china/china-20230821-5.png)
>
> #### 不使能编译器输出列表文件 
>
> 不要勾选输出编译器输出列表文件：Options > C/C++ Compiler > List：
>
> ![img](https://www.iar.com/siteassets/china/china-20230821-6.png)
>
> #### 把源文件移到本地电脑上进行Build
>
> 如果build的时候，对应源文件没有在本地电脑上，而是通过网络连接访问，建议把源文件移到本地电脑进行build，避免因为网络连接问题导致build时间很长。
>
> #### 在Windows命令行调用iarbuild命令进行Build
>
> 通常来说，在Windows命令行调用iarbuild命令进行build比在IAR Embedded Workbench IDE进行build要稍微快一些。
>
> #### 在Linux服务器上使用IAR Build Tools进行Build
>
> 前面介绍的一些方法都是基于Windows的IAR Embedded Workbench，IAR提供了可以在Linux上运行的IAR Build Tools:
>
> ![img](https://www.iar.com/siteassets/china/china-20230821-7.png)
>
> 通常来说Linux上调用IAR Build Tool进行build会比Windows的IAR Embedded Workbench进行build要快一些。下面是在同等电脑配置下Linux的IAR Build Tools和Windows的IAR Embedded Workbench的build时间对比，可以看到Linux的IAR Build Tools比Windows的IAR Embedded Workbench的build时间要降低不少。如果Linux服务器电脑配置更高，对应的build时间会降低更多。
>
> ![img](https://www.iar.com/siteassets/china/china-20230821-8.png)