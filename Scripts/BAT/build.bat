@echo off
setlocal enabledelayedexpansion

set startTime=%time%

REM IAR编译和Srec-Bin转换工具
set IarBuild="C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.4\common\bin\IarBuild.exe"
set SrecCat="C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.4\common\bin\srec_cat.exe"

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
    @echo Compile the A-partition code using the IAR toolchain
) else if %foundB%==1 (
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

if %foundA%==1 (
    @echo Generate the A-partition bin file from srec
) else if %foundB%==1 (
    @echo Generate the B-partition bin file from srec
) else (
    echo No matching definitions found
    exit /b
)

%SrecCat% %src_cm0plus_Path%cm0plus.srec -crop 0x10000000 0x1007FFFF -offset -0x10000000 -o cm0plus_app.bin -binary
%SrecCat% %src_cm7_0_Path%cm7_0.srec -crop 0x10080000 0x1027FFFF -offset -0x10080000 -o cm7_0_app.bin -binary
%SrecCat% %src_cm7_1_Path%cm7_1.srec -crop 0x10280000 0x1029BBFE -offset -0x10280000 -o cm7_1_app.bin -binary

if %foundA%==1 (
    copy %CD%\cm0plus_app.bin %OutputOrinA%\cm0plus_app.bin
    copy %CD%\cm7_0_app.bin %OutputOrinA%\cm7_0_app.bin
    copy %CD%\cm7_1_app.bin %OutputOrinA%\cm7_1_app.bin
    @echo.
    echo Switch to the B partition code
    sed -i "s/#define CURRENT_PARTITION 'A'/#define CURRENT_PARTITION 'B'/" %Cm0FilePath%
    sed -i "s/#define CURRENT_PARTITION 'A'/#define CURRENT_PARTITION 'B'/" %Cm70FilePath%
    @echo.
    @echo Compile the B-partition code using the IAR toolchain
) else if %foundB%==1 (
    copy %CD%\cm0plus_app.bin %OutputOrinB%\cm0plus_app.bin
    copy %CD%\cm7_0_app.bin %OutputOrinB%\cm7_0_app.bin
    copy %CD%\cm7_1_app.bin %OutputOrinB%\cm7_1_app.bin
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

%IarBuild% %Cm0Ewp% -clean rev_c
%IarBuild% %Cm0Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm0VarFile%
%IarBuild% %Cm70Ewp% -clean rev_c
%IarBuild% %Cm70Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm70VarFile%
%IarBuild% %Cm71Ewp% -clean rev_c
%IarBuild% %Cm71Ewp% -make rev_c -log warnings -parallel 4 -varfile %Cm71VarFile%

if %foundA%==1 (
    @echo Generate the B-partition bin file from srec
) else if %foundB%==1 (
    @echo Generate the A-partition bin file from srec
) else (
    echo No matching definitions found
    exit /b
)

%SrecCat% %src_cm0plus_Path%cm0plus.srec -crop 0x10000000 0x1007FFFF -offset -0x10000000 -o cm0plus_app.bin -binary
%SrecCat% %src_cm7_0_Path%cm7_0.srec -crop 0x10080000 0x1027FFFF -offset -0x10080000 -o cm7_0_app.bin -binary
%SrecCat% %src_cm7_1_Path%cm7_1.srec -crop 0x10280000 0x1029BBFE -offset -0x10280000 -o cm7_1_app.bin -binary

if %foundA%==1 (
    copy %CD%\cm0plus_app.bin %OutputOrinB%\cm0plus_app.bin
    copy %CD%\cm7_0_app.bin %OutputOrinB%\cm7_0_app.bin
    copy %CD%\cm7_1_app.bin %OutputOrinB%\cm7_1_app.bin
) else if %foundB%==1 (
    copy %CD%\cm0plus_app.bin %OutputOrinA%\cm0plus_app.bin
    copy %CD%\cm7_0_app.bin %OutputOrinA%\cm7_0_app.bin
    copy %CD%\cm7_1_app.bin %OutputOrinA%\cm7_1_app.bin
) else (
    echo No matching definitions found
    exit /b
)

@echo.
echo Switch back to the A partition code
sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm0FilePath%
sed -i "s/#define CURRENT_PARTITION 'B'/#define CURRENT_PARTITION 'A'/" %Cm70FilePath%

@echo.
if %errorlevel% equ 0 (
    @echo The full-link compilation is successful
) else (
    @echo The full-link compilation fails, check the path and permissions
)

@echo.
@echo Start  time: %startTime%
@echo Finish time: %time%

endlocal

pause