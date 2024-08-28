# 异步I/O+termios编程实现串口接收解释

**一、数据成员**

termios 函数族提供了一个常规的终端接口，用于控制非同步通信端口。 这个结构包含了至少下列成员： 

  tcflag_t c_iflag;    /* 输入模式 */
  tcflag_t c_oflag;    /* 输出模式 */
  tcflag_t c_cflag;    /* 控制模式 */
  tcflag_t c_lflag;    /* 本地模式 */

  cc_t c_cc[NCCS];    /* 控制字符 */



```cpp
struct termios  
{  
    unsigned short c_iflag; /* 输入模式标志*/  
    unsigned short c_oflag; /* 输出模式标志*/  
    unsigned short c_cflag; /* 控制模式标志*/  
    unsigned short c_lflag; /*区域模式标志或本地模式标志或局部模式*/  
    unsigned char c_line; /*行控制line discipline */  
    unsigned char c_cc[NCC]; /* 控制字符特性*/  
};  
```

**二、作用**
  这个变量被用来提供一个健全的线路设置集 合, 如果这个端口在被用户初始化前使用. 驱动初始化这个变量使用一个标准的数值集, 它拷贝自 tty_std_termios变量. tty_std_termos 在 tty 核心被定义为:

```cpp
struct termios tty_std_termios = {  
    .c_iflag = ICRNL | IXON,  
    .c_oflag = OPOST | ONLCR,  
    .c_cflag = B38400 | CS8 | CREAD | HUPCL,  
    .c_lflag = ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | IEXTEN,  
    .c_cc = INIT_C_CC  
}; 
```

  这个 struct termios 结构用来持有所有的当前线路设置, 给这个 tty 设备的一个特定端口. 这些线路设置控制当前波特率, 数据大小, 数据流控设置, 以及许多其他值. 



**三、成员的值**
  （一）c_iflag 标志常量：Input mode ( 输入模式)
input mode可以在输入值传给程序之前控制其处理的方式。其中输入值可能是由序列埠或键盘的终端驱动程序所接收到的字元。
我们可以利用termios结构的c_iflag的标志来加以控制，其定义的方式皆以OR来加以组合。
  \* IGNBRK ：忽略输入中的 BREAK 状态。 （忽略命令行中的中断） 
  \* BRKINT ：（命令行出 现中断时，可产生一插断）如果设置了GNBRK，将忽略 BREAK。如果没有设置，但是设置了 BRKINT，那么 BREAK 将使得输入和输出队列被刷新，如果终端是一个前台进程组的控制终端，这个进程组中所有进程将收到 SIGINT信号。如果既未设置IGNBRK也未设置BRKINT，BREAK 将视为与NUL字符同义，除非设置了PARMRK，这种情况下它被视为序列377 &#0; &#0;。  
  \* IGNPAR ：忽略桢错误和奇偶校验错。  
  \* PARMRK ：如果没有设置 IGNPAR，在有奇偶校验错或桢错误的字符前插入377 &#0;。如果既没有设置 IGNPAR 也没有设置PARMRK，将有奇偶校验错或桢错误的字符视为 &#0;。  
  \* INPCK ：启用输入奇偶检测。  
  \* ISTRIP ：去掉第八位。  
  \* INLCR :将输入中的 NL 翻译为 CR。（将收到 的换行符号转换为Return）  
  \* IGNCR :忽略输入中的回车。  
  \* ICRNL :将输入中的回车翻译为新行 (除非设置了 IGNCR)(否则当输入信号有 CR 时不会终止输入)。  
  \* IUCLC :(不属于 POSIX) 将输入中的大写字母映射为小写字母。  
  \* IXON :启用输出的 XON/XOFF 流控制。  
  \* IXANY :(不属于 POSIX.1；XSI) 允许任何字符来重新开始输出。(?)  
  \* IXOFF :启用输入的 XON/XOFF 流控制。  
  \* IMAXBEL:(不属于 POSIX) 当输入队列满时响零。Linux 没有实现这一位，总是将它视为已设置。 

  （二） c_oflag 标志常量：Output mode ( 输 出模式)
Output mode主要负责控制输出字元的处理方式。输出字元在传送到序列埠或显示器之前是如何被程序来处理。
输出模式是利用termios结构的c_oflag的标志来加以控制，其定义的方式皆以OR来加以组合。 
  \* OPOST ：启用具体实现自行定义的输出处理。  
  \* OLCUC ：(不属于 POSIX) 将输出中的小写字母映射为大写字母。  
  \* ONLCR ：(XSI) 将输出中的新行符映射为回车-换行。  
  \* OCRNL ：将输出中的回车映射为新行符  
  \* ONOCR ：不在第 0 列输出回车。  
  \* ONLRET ：不输出回车。  
  \* OFILL ：发送填充字符作为延时，而不是使用定时来延时。  
  \* OFDEL ：(不属于 POSIX) 填充字符是 ASCII DEL (0177)。如果不设置，填充字符则是 ASCII NUL。  
  \* NLDLY ：新行延时掩码。取值为 NL0 和 NL1。  
  \* CRDLY ：回车延时掩码。取值为 CR0, CR1, CR2, 或 CR3。  
  \* TABDLY ：水平跳格延时掩码。取值为 TAB0, TAB1, TAB2, TAB3(或 XTABS)。取值为 TAB3，即 XTABS，将扩展跳格为空格 (每个跳格符填充 8 个空格)。(?)  
  \* BSDLY ：回退延时掩码。取值为 BS0 或 BS1。(从来没有被实现过)  
  \* VTDLY ：竖直跳格延时掩码。取值为 VT0 或 VT1。  
  \* FFDLY ：进表延时掩码。取值为 FF0 或 FF1。

  （三） c_cflag 标志常量：Control mode ( 控制模式)
Control mode主要用于控制终端设备的硬件设置。利用termios结构的c_cflag的标志来加以控制。控制模式用在序列线连接到数据设备，也可以用在与终 端设备的交谈。
一般来说，改变终端设备的组 态要比使用termios的控制模式来改变行(lines)的行为来得容易。 
  \* CBAUD ：(不属于 POSIX) 波特率掩码 (4+1 位)。  
  \* CBAUDEX ：(不属于 POSIX) 扩展的波特率掩码 (1 位)，包含在CBAUD 中。  
  \* (POSIX 规定波特率存储在 termios 结构中，并未精确指定它的位置，而是提供了函数 cfgetispeed() 和 cfsetispeed() 来存取它。一些系统使用 c_cflag 中 CBAUD 选择的位，其他系统使用单独的变量，例如 sg_ispeed 和 sg_ospeed 。)  
  \* CSIZE：字符长度掩码（传送或接收字元时用的位数）。 取值为CS5（传送或接收字元时用5bits）, CS6, CS7, 或 CS8。  
  \* CSTOPB ：设置两个停止位，而不是一个。  
  \* CREAD ：打开接受者。  
  \* PARENB ：允许输出产生奇偶信息以及输入的奇偶校验（启用同位产生与侦测）。  
  \* PARODD ：输入和输出是奇校验（使用奇同位而非偶同位）。  
  \* HUPCL ：在最后一个进程关闭设备后，降低 modem 控制线 (挂断)。(?)  
  \* CLOCAL ：忽略 modem 控制线。  
  \* LOBLK :(不属于 POSIX) 从非当前 shell 层阻塞输出(用于shl )。(?)  
  \* CIBAUD :(不属于 POSIX) 输入速度的掩码。CIBAUD 各位的值与CBAUD 各位相同，左移了 IBSHIFT 位。  
  \* CRTSCTS :(不属于 POSIX) 启用 RTS/CTS (硬件) 流控制。

  （四） c_lflag 标志常量：Local mode ( 局部模式)
Local mode主要用来控制终端设备不同的特色。利用termios结构里的c_lflag的标志来设定局部模式。在巨集中有两个比较重要的标志：
  *ECHO：它可以让你阻止键入字元的回应。
  *ICANON(正规模式)标志，它可以对所接收的字元在两种不同的终端设备模式之间来回切 换。 
  \* ISIG：当接受到字符 INTR, QUIT, SUSP, 或 DSUSP 时，产生相应的信号。  
  \* ICANON：启用标准模式 (canonical mode)。允许使用特殊字符EOF, EOL, EOL2, ERASE, KILL, LNEXT, REPRINT, STATUS, 和WERASE，以及按行的缓冲。  
  \* XCASE:(不属于 POSIX; Linux 下不被支持) 如果同时设置了ICANON，终端只有大写。输入被转换为小写，除了有前缀的字符。输出时，大写字符被前缀（某些系统指定的特定字符） ，小写字符被转换成大写。  
  \* ECHO ：回显输入字符。  
  \* ECHOE ：如果同时设置了 ICANON，字符 ERASE 擦除前一个输入字符，WERASE 擦除前一个词。  
  \* ECHOK ：如果同时设置了 ICANON，字符 KILL 删除当前行。  
  \* ECHONL ：如果同时设置了 ICANON，回显字符 NL，即使没有设置 ECHO。  
  \* ECHOCTL ：(不属于 POSIX) 如果同时设置了 ECHO，除了 TAB,NL, START, 和 STOP 之外的 ASCII 控制信号被回显为 ^X, 这里 X 是比控制信号大 0x40 的 ASCII 码。例如，字符0x08(BS) 被回显为 ^H。  
  \* ECHOPRT ：(不属于 POSIX) 如果同时设置了 ICANON 和IECHO，字符在删除的同时被打印。  
  \* ECHOKE ：(不属于 POSIX) 如果同时设置了 ICANON，回显 KILL时将删除一行中的每个字符，如同指定了 ECHOE 和 ECHOPRT 一样。  
  \* DEFECHO :(不属于 POSIX) 只在一个进程读的时候回显。  
  \* FLUSHO :(不属于 POSIX; Linux 下不被支持) 输出被刷新。这个标志可以通过键入字符 DISCARD 来开关。  
  \* NOFLSH :禁止在产生 SIGINT, SIGQUIT 和 SIGSUSP 信号时刷新输入和输出队列，即关闭queue中的flush。 
  \* TOSTOP :向试图写控制终端的后台进程组发送 SIGTTOU 信号（传送欲写入的信息到后台 处理）。  
  \* PENDIN :(不属于 POSIX; Linux 下不被支持) 在读入下一个字符时，输入队列中所有字符被重新输出。(bash 用它来处理typeahead)  
  \* IEXTEN :启用实现自定义的输入处理。这个标志必须与 ICANON同时使用，才能解释特殊字符 EOL2，LNEXT，REPRINT 和WERASE，IUCLC 标志才有效。 

  ***\*（五） c_cc 数组：特殊控制字元可提供使用者设定一些特殊的功能，如Ctrl+C的字元组合。\****

特殊控制字元主要是利用termios结构里c_cc的阵列成员 来做设定。c_cc阵列主要用于正规与非正规两种环境，但要注意的是正规与非正规不可混为一谈。其定义了特殊的控制字符。符号下标 (初始值) 和意义为： 

  \* VINTR：(003, ETX, Ctrl-C, or also 0177, DEL, rubout) 中断字符。发出 SIGINT 信号。当设置 ISIG 时可被识别，不再作为输入传递。  
  \* VQUIT ：(034, FS, Ctrl-) 退出字符。发出 SIGQUIT 信号。当设置 ISIG 时可被识别，不再作为输入传递。  
  \* VERASE ：(0177, DEL, rubout, or 010, BS, Ctrl-H, or also#) 删除字符。删除上一个还没有删掉的字符，但不删除上一个EOF 或行首。当设置 ICANON 时可被识别，不再作为输入传递。  
  \* VKILL ：(025, NAK, Ctrl-U, or Ctrl-X, or also @) 终止字符。删除自上一个 EOF 或行首以来的输入。当设置 ICANON 时可被识别，不再作为输入传递。  
  \* VEOF ：(004, EOT, Ctrl-D) 文件尾字符。更精确地说，这个字符使得 tty 缓冲中的内容被送到等待输入的用户程序中，而不必等到 EOL。如果它是一行的第一个字符，那么用户程序的read() 将返回 0，指示读到了 EOF。当设置 ICANON 时可被识别，不再作为输入传递。  
  \* VMIN :非 canonical 模式读的最小字符数（MIN 主要是表示能满足read的最小字元数）。  
  \* VEOL :(0, NUL) 附加的行尾字符。当设置 ICANON 时可被识别。  
  \* VTIME ：非 canonical 模式读时的延时，以十分之一秒为单位。  
  \* VEOL2 ：(not in POSIX; 0, NUL) 另一个行尾字符。当设置ICANON 时可被识别。  
  \* VSWTCH ：(not in POSIX; not supported under Linux; 0,NUL) 开关字符。(只为 shl 所用。)  
  \* VSTART ：(021, DC1, Ctrl-Q) 开始字符。重新开始被 Stop 字符中止的输出。当设置 IXON 时可被识别，不再作为输入传递。  
  \* VSTOP ：(023, DC3, Ctrl-S) 停止字符。停止输出，直到键入Start 字符。当设置 IXON 时可被识别，不再作为输入传递。  
  \* VSUSP ：(032, SUB, Ctrl-Z) 挂起字符。发送 SIGTSTP 信号。当设置 ISIG 时可被识别，不再作为输入传递。 
  \* VDSUSP ：(not in POSIX; not supported under Linux; 031,EM, Ctrl-Y) 延时挂起信号。当用户程序读到这个字符时，发送SIGTSTP 信号。当设置 IEXTEN 和 ISIG，并且系统支持作业管理时可被识别，不再作为输入传递。  
  \* VLNEXT ：(not in POSIX; 026, SYN, Ctrl-V) 字面上的下一个。引用下一个输入字符，取消它的任何特殊含义。当设置IEXTEN 时可被识别，不再作为输入传递。  
  \* VWERASE ：(not in POSIX; 027, ETB, Ctrl-W) 删除词。当设置 ICANON 和 IEXTEN 时可被识别，不再作为输入传递。  
  \* VREPRINT ：(not in POSIX; 022, DC2, Ctrl-R) 重新输出未读的字符。当设置 ICANON 和 IEXTEN 时可被识别，不再作为输入传递。  
  \* VDISCARD ：(not in POSIX; not supported under Linux;017, SI, Ctrl-O) 开关：开始/结束丢弃未完成的输出。当设置IEXTEN 时可被识别，不再作为输入传递。  
  \* VSTATUS ：(not in POSIX; not supported under Linux;status request: 024, DC4, Ctrl-T).  
  \* 这些符号下标值是互不相同的，除了 VTIME，VMIN 的值可能分别与 VEOL，VEOF 相同。 (在 non-canonical 模式下，特殊字符的含义更改为延时含义。MIN 表示应当被读入的最小字符数。TIME 是以十分之一秒为单位的计时器。如果同时设置了它们，read 将等待直到至少读入一个字符，一旦读入 MIN 个字符或者
从上次读入字符开始经过了 TIME 时间就立即返回。如果只设置了 MIN，read 在读入 MIN 个字符之前不会返回。如果只设置了TIME，read 将在至少读入一个字符，或者计时器超时的时候立即返回。如果都没有设置，read 将立即返回，只给出当前准备好的字符。)MIN与 TIME组合有以下四种：
  1、 MIN = 0 , TIME =0 有READ立即回传否则传回 0 ,不读取任何字元
  2、 MIN = 0 , TIME >0 READ 传回读到的字元,或在十分之一秒后传回TIME若来不及读到任何字元,则传回0
  3、 MIN > 0 , TIME =0 READ 会等待,直到MIN字元可读
  4、 MIN > 0 , TIME > 0 每一格字元之间计时器即会被启动READ 会在读到MIN字元,传回值或TIME的字元计时(1/10秒)超过时将值 传回

**四、 与此结构体相关的函数**
（一）tcgetattr()
  1.原型 

```cpp
int tcgetattr(int fd,struct termois & termios_p);
```

  2.功能 
取得终端介质（fd）初始值，并把其值 赋给temios_p;函数可以从后台进程中调用；但是，终端属性可能被后来的前 台进程所改变。 
（二）tcsetattr() 
  1.原型

```cpp
int tcsetattr(int fd,int actions,const struct termios *termios_p);
```

  2.功能
  设置与终端相关的参数 (除非需要底层支持却无法满足)，使用termios_p 引用的 termios 结构。optional_actions （tcsetattr函数的第二个参数）指定了什么时候改变会起作用： 
  \* TCSANOW：改变立即发生  
  \* TCSADRAIN：改变在所有写入 fd 的输出都被传输后生效。这个函数应当用于修改影响输出的参数时使用。(当前输出完成时将值改变)  
  \* TCSAFLUSH ：改变在所有写入 fd 引用的对象的输出都被传输后生效，所有已接受但未读入的输入都在改变发生前丢弃(同TCSADRAIN，但会舍弃当前所有值)。 

（三）tcsendbreak()

  传送连续的 0 值比特流，持续一段时间，如果终端使用异步串行数据传输的话。如果 duration 是 0，它至少传输 0.25 秒，不会超过 0.5 秒。如果duration 非零，它发送的时间长度由实现定义。如果终端并非使用异步串行数据传输，tcsendbreak() 什么都不做。 

（四）tcdrain() 

  等待直到所有写入 fd 引用的对象的输出都被传输。 

（五）tcflush() 
  丢弃要写入引用的对象，但是尚未传输的数据，或者收到但是尚未读取的数据，取决于 queue_selector 的值： 
  \* TCIFLUSH ：刷新收到的数据但是不读  
  \* TCOFLUSH ：刷新写入的数据但是不传送  
  \* TCIOFLUSH ：同时刷新收到的数据但是不读，并且刷新写入的数据但是不传送 

（六）tcflow() 
  挂起 fd 引用的对象上的数据传输或接收，取决于 action 的值： 
  \* TCOOFF ：挂起输出  
  \* TCOON ：重新开始被挂起的输出  
  \* TCIOFF ：发送一个 STOP 字符，停止终端设备向系统传送数据  
  \* TCION ：发送一个 START 字符，使终端设备向系统传输数据打开一个终端设备时的默认设置是输入和输出都没有挂起。
（七） 波特率函数 
  被用来获取和设置 termios 结构中，输入和输出波特率的值。新值不会马上生效，直到成功调用了 tcsetattr() 函数。设置速度为 B0 使得 modem "挂机"。与 B38400 相应的实际比特率可以用setserial(8) 调整。 输入和输出波特率被保存于 termios 结构中。 cfmakeraw 设置终端属性如下： 

```cpp
termios_p->c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);  
termios_p->c_oflag &= ~OPOST;  
termios_p->c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);  
termios_p->c_cflag &= ~(CSIZE|PARENB);  
termios_p->c_cflag |= CS8;
```

  *cfgetospeed() 返回 termios_p 指向的 termios 结构中存储的输出波特率 
  *cfsetospeed() 设置 termios_p 指向的 termios 结构中存储的输出波特率为speed。取值必须是以下常量之一： 

```cpp
B0        B50        B75        B110        B134        B150  
B200      B300       B600       B1200       B1800  
B2400     B4800      B9600      B19200      B38400  
B57600    B115200    B230400  </span>
```

其中：零值 B0 用来中断连接。如果指定了 B0，不应当再假定存在连接。通常，这样将断开连接。CBAUDEX 是一个掩码，指示高于 POSIX.1 定义的速度的那一些(57600 及以上)。因此，B57600 & CBAUDEX 为非零。 
  *cfgetispeed() 返回 termios 结构中存储的输入波特率。 
  *cfsetispeed() 设置 termios 结构中存储的输入波特率为 speed。如果输入波特率被设为0，实际输入波特率将等于输出波特率。 

**五、 RETURN VALUE 返回值**
  *cfgetispeed() 返回 termios 结构中存储的输入波特率。 
  *cfgetospeed() 返回 termios 结构中存储的输出波特率。 
 其他函数返回： 
  0：成 功 
  -1:失 败，并且为 errno 置值来指示错误。注意 tcsetattr() 返回成功，如果任何所要求的修改可以实现的话。因此，当进行多重修改时，应当在这个函数之后再次调用 tcgetattr() 来检测是否所有修改都成功实现。 

***\*六、NOTES 注意\****

  Unix V7 以及很多后来的系统有一个波特率的列表，在十四个值 B0, ..., B9600之后可以看到两个常数 EXTA, EXTB ("External A" and "External B")。很多系统将这个列表扩展为更高的波特率。tcsendbreak 中非零的 duration 有不同的效果。SunOS 指定中断 duration*N秒，其中 N 至少为 0.25，不高于 0.5 。Linux, AIX, DU, Tru64 发送 duration微秒的 break 。FreeBSD, NetBSD, HP-UX 以及 MacOS 忽略 duration 的值。在Solaris 和 Unixware 中， tcsendbreak 搭配非零的 duration 效果类似于tcdrain。 



termios是面向所有终端设备的。
termios 结构体：

```c
   tcflag_t c_iflag;      /* input modes */
   tcflag_t c_oflag;      /* output modes */
   tcflag_t c_cflag;      /* control modes */
   tcflag_t c_lflag;      /* local modes */
   cc_t     c_cc[NCCS];   /* special characters */
```

### 终端的三种模式

**规范模式（命令行的形式）**
所有输入基于行进行处理。在用户输入一个行结束符（回车符、EOF等）之前，系统调用`read()`函数读不到用户输入的任何字符。其次，除了EOF之外的行结束符与普通字符一样会被`read()`函数读取到缓冲区中。一次调用`read()`只能读取一行数据。
**非规范模式**
所有输入是即时有效的，不需要用户另外输入行结束符。
**原始模式**
是一种特殊的非规范模式，所有的输入数据以字节为单位被处理。即有一个字节输入时，触发输入有效。

但是串口并不仅仅只扮演着人机交互的角色（数据以字符的形式传输、也就数说传输的数据其实字符对应的 ASCII 编码值）；串口本就是一种数据串行传输接口，通过串口可以与其他设备或传感器进行数据传输、通信，譬如很多 sensor 就使用了串口方式与主机端进行数据交互。那么在这种情况下，我们就得使用原始模式，意味着通过串口传输的数据不应进行任何特殊处理、不应将其解析成 ASCII 字符。

### 终端控制API函数

```c
tcgetattr      取属性(termios结构)
tcsetattr      设置属性(termios结构)
cfgetispeed    得到输入速度
cfgetospeed    得到输出速度
cfsetispeed    设置输入速度
cfsetospeed    设置输出速度
tcdrain        等待所有输出都被传输
tcflow         挂起传输或接收
tcflush        刷清未决输入和/或输出
tcsendbreak    送BREAK字符
tcgetpgrp      得到前台进程组ID
tcsetpgrp      设置前台进程组ID
cfmakeraw    将终端设置成原始模式
cfsetspeed   设置输入输出速度
```

需要注意的地方：

```c
fd = open(device,O_RDWR|O_NOCTTY|O_NDELAY);
```

1. `O_NONBLOCK`/如果`pathname`指的是一个FIFO、一个块特殊文件或一个字符特殊文件，则此选择项为此文件的本次打开操作和后续的I/O操作设置非阻塞方式。
2. `O_NOCTTY` 如果`pathname`指的是终端设备，则不将此设备分配作为此进程的控制终端。（个人的理解是只有read和write才能对指定此终端设备进行通信）。
3. `O_NONBLOCK`和`O_NDELAY`几乎相同，它们的差别在于设立`O_NDELAY`会使I/O函式马上回传0，但是又衍生出一个问题，因为读取到档案结尾时所回传的也是0，这样无法得知是哪中情况；因此，`O_NONBLOCK`就产生出来，它在读取不到数据时会回传-1，并且设置errno为`EAGAIN`。
4. fd是int类型，device是char*指针，eg：“`/dev/ttyUSB0`”。

```c
tcgetattr(fd, &old_cfg)；
```

获取终端设备的参数，保存至old_cfg中，old_cfg是自己设定的全局变量，类型是struct termios。old_cfg的作用，退出主循环后，还要将终端设备的工作模式恢复成规范模式（命令行模式解析成ASCII码）。

```c
tcsetattr(fd, TCSANOW, &old_cfg);
close(fd);
```

中间的参数是配置立即生效

```C
cfmakeraw(&new_cfg);
```

将终端设备设置为原始模式

```C
new_cfg.c_cflag |= CREAD;	//接收模式
cfsetspeed(&new_cfg,speed)
```

设置输入输出baud率，speed是speed_t类型的

```C
new_cfg.c_cflag &= ~CSIZE; //清空数据位控制字
new_cfg.c_cflag |= CS8;		//数据位八位
new_cfg.c_cflag &= ~PARENB;	//配置为无校验
new_cfg.c_iflag &= ~INPCK;		//配置为无校验
new_cfg.c_cflag &= ~CSTOPB;	//一个停止位，如果是或上就是两个停止位
new_cfg.c_cc[VTIME] = 0;
new_cfg.c_cc[VMIN] = 0;
```

在对接收字符和等待时间没有特别要求的情况下，可以将 MIN 和 TIME 设置为 0，这样则在任何情况下 read()调用都会立即返回，此时对串口的 read 操作会设置为非阻塞方式

### 异步I/O配置使用

类似于使用中断

```c
int flag;
flag = fcntl(fd, F_GETFL);             //先获取原来的 flag
flag |= O_ASYNC;                         //将 O_ASYNC 标志添加到  flag 
fcntl(fd, F_SETFL, flag);               //重新设置  flag
```

```c
fcntl(fd, F_SETOWN, getpid());  
```

为文件描述符设置异步 I/O 事件的接收进程，也就是设置异步 I/O 的所有者。

```c
fcntl(fd, F_SETSIG, SIGRTMIN); 
```

`SIGRTMIN`编号是34，小于34的都是不可靠信号。
`SIGRTMAX`编号是64号。
指定实时信号SIGRTMIN作为异步I/O通知信号。 `SIGRTMIN->(SIG-REAL-TIME-MINIMUM)`是实时信号
注意： `F_SETSIG`要使用`#define _GNU_SOURCE` 宏定义

```c
struct sigaction act;
act.sa_sigaction = io_handler; //sa_sigaction是个函数指针，指向相应的处理函数
act.sa_flags = SA_SIGINFO; 
sigemptyset(&act.sa_mask); 
sigaction(SIGRTMIN, &act, NULL);
```

下面是例程代码

```c
#define _GNU_SOURCE //在源文件开头定义_GNU_SOURCE宏
#include <stdio.h> 
#include <stdlib.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <unistd.h> 
#include <sys/ioctl.h> 
#include <errno.h> 
#include <string.h> 
#include <signal.h> 
#include <termios.h>

typedef struct uart_cfg_t
{
    unsigned int baudrate;
    unsigned char databit;
    char parity;
    unsigned char stopbit;
}
uart_cfg;

static struct termios old_cfg;
static int fd;
char *device = "/dev/ttyUSB0";

static int uart_init(const char *device);
static int uart_config(const uart_cfg *cfg);
static void async_io_init(void);
static void io_handler(int sig,siginfo_t *info, void *context);

int main(int argc, char *argv[])
{
    uart_cfg cfg = {0};
    int ret;
    uart_init(device);
    cfg.baudrate = 115200;
    ret = uart_config(&cfg);
    if(ret)
    {
        tcsetattr(fd, TCSANOW, &old_cfg);
        close(fd);
        exit(EXIT_FAILURE);
    }
    async_io_init();
    while(1)
    {
        sleep(1);
    }
    tcsetattr(fd, TCSANOW, &old_cfg); //恢复到之前的配置 
    close(fd); 
    exit(EXIT_SUCCESS);

}


/*异步IO初始化函数*/
static void async_io_init(void)
{
    struct sigaction sig;
    int flag;

/*使能异步I/O*/            /**/
flag = fcntl(fd,F_GETFL); /*使能串口的异步I/O功能*/
flag |= O_ASYNC;
fcntl(fd,F_SETFL,flag);
/*设置异步I/O的所有者*/
fcntl(fd,F_SETOWN,getpid());
/*为实时信号SIGRTMIN作为异步I/O信号*/
fcntl(fd,F_SETSIG,SIGRTMIN);

sig.sa_sigaction = io_handler;
sig.sa_flags = SA_SIGINFO;
sigisemptyset(&sig.sa_mask);
sigaction(SIGRTMIN,&sig,NULL);

}

/*UART配置*/
static int uart_config(const uart_cfg *cfg)
{
    struct termios new_cfg = {0};
    speed_t speed;
    cfmakeraw(&new_cfg);
    new_cfg.c_cflag |= CREAD;

switch(cfg->baudrate)
{
    case 9600: speed = B9600; 
    break;
    case 38400: speed = B38400;
    break;
    case 57600: speed = B57600;
    break;
    case 115200: speed = B115200;
    break;
    default: 
        speed = B115200;
        printf("default baud rate: 115200\n");
        break;
}

if(0 > cfsetspeed(&new_cfg,speed))
{
    fprintf(stderr,"cfsetspeed error: %s\n)",strerror(errno));
}
    new_cfg.c_cflag &= ~CSIZE;
    new_cfg.c_cflag |= CS8;
    new_cfg.c_cflag &= ~PARENB;
    new_cfg.c_iflag &= ~INPCK;
    new_cfg.c_cflag &= ~CSTOPB;

new_cfg.c_cc[VTIME] = 0;
new_cfg.c_cc[VMIN] = 0;

if(0 > tcflush(fd,TCIOFLUSH))
{
    fprintf(stderr,"tcflush error:%s\n",strerror(errno));
    return -1;
}

if(0 > tcsetattr(fd,TCSANOW,&new_cfg))
{
    fprintf(stderr,"tcsetattr error: %s\n",strerror(errno));
    return -1;
}
return 0;

}

/*UART初始化*/
static int uart_init(const char *device)
{
    fd = open(device,O_RDWR|O_NOCTTY|O_NDELAY);
    if(0 > fd)
    {
        fprintf(stderr, "open error : %s: %s\n",device,strerror(errno));
        return -1;        
    }

if(0 > tcgetattr(fd, &old_cfg))
{
    fprintf(stderr, "tcgetattr error : %s\n",strerror(errno));
    close(fd);
    return -1;
}
return 0;

}

static void io_handler(int sig,siginfo_t *info, void *context)
{
    unsigned char buf[128] = {0};
    int ret;
    int n;

if(SIGRTMIN != sig)
{
    return;
}

if(POLL_IN == info->si_code)
{
    ret = read(fd,buf,128);
    for(n = 0;n < ret ;n++)
    {

//            printf("0x%hhx ",buf[n]);
            printf("%hhc",buf[n]);
        }
    }
}
```

注意：

```c
ret = read(fd,buf,128);
```

ret返回的值是读取到接收缓冲区中的数据长度。

```C
sig.sa_flags = SA_SIGINFO;
```

`SA_SIGINFO`控制字代表`sa_sigaction`成员有效，`sa_handler`成员无效。
`sa_sigaction`和`sa_handler`都是函数指针。
区别是`sa_sigaction`的参数信息更多。