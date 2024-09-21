@echo off

REM 设置源文件和目标文件路径
set SRC_FILE=main.c
set OUT_FILE=main.out

REM 使用gcc编译源文件生成.out可执行文件
gcc %SRC_FILE% -o %OUT_FILE%

REM 检查编译是否成功
if %ERRORLEVEL% neq 0 (
    echo 编译失败，退出脚本...
    exit /b %ERRORLEVEL%
)

REM 复制生成的.out文件到目标目录
set DEST_DIR=Y:\MCU_BSP\BIN_Dev
if not exist %DEST_DIR% (
    echo 目标目录不存在，创建目录...
    mkdir %DEST_DIR%
)

copy %OUT_FILE% %DEST_DIR%

echo 编译和复制完成！
