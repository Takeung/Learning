那就是result[16]=(R/(100+R))*3.3/4096，R=R0*100/(R0+100)，R就是热敏电阻R0和100K电阻并联后的电阻值，R0是热敏电阻值，然后反求R0->查表得到温度，result[16]就是测得后转换的电压值

![](https://s2.loli.net/2024/09/19/eOjwfq6g9QCHFax.png)

计算公式：

![](https://s2.loli.net/2024/09/19/6DUe3S8CmKTLliR.png)

> 注：上图中右下公式多除了4096

读取ADC的原始值

![](https://s2.loli.net/2024/09/19/da42kYRspHnI5uJ.jpg)

读ADC并转换为实际电压后：

![](https://s2.loli.net/2024/09/19/IOdebm3kSh5J8Yy.jpg)