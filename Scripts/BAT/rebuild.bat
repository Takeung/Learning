echo =================先清除=======================
 
"C:/Program Files (x86)/IAR Systems/Embedded Workbench 8.1/common/bin/IarBuild.exe" Test.ewp -clean Debug -log all
 
del 生产烧录文件\Test_MCU.bin
del 生产烧录文件\Test_SpiFlash.bin
 
 
echo =================清除完成，请检查【生产烧录文件】是否清空，接下来开始编译=======================
 
timeout /t 10
 
echo =================开始编译======================= 
"C:/Program Files (x86)/IAR Systems/Embedded Workbench 8.1/common/bin/IarBuild.exe" Test.ewp -make Debug -log all
 
echo =================编译完成,接下来复制bin文件到最终目录=======================
 
copy Debug\Exe\Test.bin 生产烧录文件\Test_MCU.bin
 
echo =================复制MCU bin文件到【生产烧录文件】完成=======================
 
copy ..\..\soft_pc\SpiFlash文件\SpiFlash.bin 生产烧录文件\Test_SpiFlash.bin
 
echo =================复制SPI FLASH 文件到【生产烧录文件】完成=======================
echo ################ 恭喜您，所有流程都完成了，请退出。 #####################
 
timeout /t 60