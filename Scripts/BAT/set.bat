@echo off
setlocal enabledelayedexpansion

REM 定义文件路径
set Cm0FilePath=main_cm0plus.c
set Cm70FilePath=hardware\Timer\user_timer.c

REM 初始化标志位
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

REM 根据找到的内容进行替换
if %foundA%==1 (
    echo Switch to the B partition code
    REM 使用 sed 替换内容
    sed -i "s/#define CURRENT_PARTITION 'A'/#define CURRENT_PARTITION 'B'/" %Cm0FilePath%
    sed -i "s/#define CURRENT_PARTITION 'A'/#define CURRENT_PARTITION 'B'/" %Cm70FilePath%
) else if %foundB%==1 (
    echo Switch to the A partition code
    REM 使用 sed 替换内容
    sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm0FilePath%
    sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm70FilePath%
) else (
    echo No matching definitions found
    exit /b
)

echo Partition code switch completed.

endlocal