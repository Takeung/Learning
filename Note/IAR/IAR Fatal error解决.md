# IAR Fatal error解决

## 错误描述

打开IAR工程或者编译后弹出如下错误
Fatal error while generating source browse infomation.See the Source Browse Log window for more infomation

![img](https://img-blog.csdnimg.cn/20200521102625203.png)

同时查看IAR Source Browser Log，显示**Failed to parse input files**的问题

![img](https://img-blog.csdnimg.cn/20200521103041977.png)

## 错误现象

工程各个源文件之间没关联，函数，变量，声明等无法跳转。这是由于工程各个源文件之间无法关联，无法建立依赖关系，即无法Generating browse information

## 错误解决

两种可能原因：

- **中文路径问题**：将工程目录内所有路径设置成英文路径，不能包含中文及其他编码格式

     实测发现即便路径名中存在空格或者“.”，并不会影响路径文件的正常解析

- **工程更新问题**：这种情况主要出现在工程拷贝过程中，比如从一台PC机拷贝至另一台后很容易出现。

     由于IAR编译机制问题（速度更快，更省时），当源文件未更新时，中间编译过程文件 .o等，在下次编译时并不会更新，即便是使用clean或rebuild all命令也是一样的，而Generating browse information建立各个源文件之间的关联需要这些中间文件（使用跳转命令 go to defintion的前提），因为未更新，所以还是使用的拷贝前的路径去查找，导致关联失败。

​		这种情况解决办法就是：把IAR工程目录内List 和Obj文件夹清空，再重新编译即可

![img](https://img-blog.csdnimg.cn/20200521104040586.png)

​		实际上，在remake之前进行clean操作能够快速完成中间编译过程文件的清理。

- **文件格式问题**：比如在Linux下存储的工程文件，拷贝到windows环境下直接使用IAR工具打开，可能会因为格式不匹配的问题导致解析失败。此时重新拷贝原工程到windows下再解压即可正常解析。