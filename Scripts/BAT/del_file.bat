@echo off

REM 定义要删除的文件路径
set FILE_TO_DELETE=Y:\MCU_BSP\BIN_Dev\main.out

REM 删除文件
del /F /Q %FILE_TO_DELETE%

REM 检查文件是否删除成功
if exist %FILE_TO_DELETE% (
    echo 文件删除失败！
) else (
    echo 文件删除成功！
)
