> 摘录于：https://blog.csdn.net/booksyhay/article/details/121105145

### 默认格式化工具 

打开设置(Ctrl + ,) ，选择“用户”配置，找到“文本编辑器” - “Default Formatter”：

![img](https://i-blog.csdnimg.cn/blog_migrate/1037a159edff5b4a5ea15b1c001fa021.png)

安装了C/C++插件后，即可选择：C/C++ ms-vscode.cpptools

#### 键入时自动格式化 

建议使能该选项：设置->文本编辑器->格式化->Format on Type:

勾选后，当敲回车键时即可看到格式化的效果。

![img](https://i-blog.csdnimg.cn/blog_migrate/840e2185b6f3d3b96b73451f7ee4c1a2.png)

#### C/C++格式化

 在用户 - 扩展 - C/C++ 中，找到C_Cpp: Formatting。选择“vcFormat”

![img](https://i-blog.csdnimg.cn/blog_migrate/f45806f6f493b83cce075b27ac3a8ce4.png)

这里也可以选择“clangFormat”。

#### Clang格式化风格

当上一步选择“clangFormat”后，需要进一步设置clang的风格：Clang_format_style。

![img](https://i-blog.csdnimg.cn/blog_migrate/ac9f1c0b02613e80ed3f59ce22edf7dd.png)

可选项有：Visual Studio, LLM, Google等。常用的还有"file"，即使用用户自定义的.clang-format文件中的配置。 

当找不到用户自定义的.clang-format配置文件时，还有个备选方案：

![img](https://i-blog.csdnimg.cn/blog_migrate/61efda50459914d5c214518d68e66695.png)

 

### vcFormat

下面重点说一下vcFormat的格式。当前面的C_Cpp: Formatting配置中选择vcFormat时生效。

主要有以下三大类：缩进，新行，空格。

#### 缩进Indent

C_Cpp配置中，以vc Format > Indent开头的。

![img](https://i-blog.csdnimg.cn/blog_migrate/52b8bd03d7ebff863358f06273828343.png)

 保持默认配置即可。

### 新行New Line

配置->扩展->C/C++中，以“vc Format > New Line”开头的配置：

![img](https://i-blog.csdnimg.cn/blog_migrate/f941c0b1d889813efa95c3bd25e4c676.png)

常用的选项主要是：

#### newLine：将左大括号移动到新行：

![img](https://i-blog.csdnimg.cn/blog_migrate/cd997118bfc772c92f634b35bf46bcb5.png)


效果：

![img](https://i-blog.csdnimg.cn/blog_migrate/f9be5d6ff09f84e98e6c8fe29e47bf76.png)

####  sameLine同一行

 左大括号保留在同一行上，并有每个左大括号的前面添加一个空格。

![img](https://i-blog.csdnimg.cn/blog_migrate/26bd25ebef6d493b600a938f30d4269b.png)

效果：

![img](https://i-blog.csdnimg.cn/blog_migrate/afb5fed1272d1c9629a6075a68564e13.png)

#### ignore忽略

即：不自动格式化，输入什么就是什么。

#### Space空格

配置->扩展->C/C++中，以“vc Format > Space”开头的配置：

![img](https://i-blog.csdnimg.cn/blog_migrate/068ac7b18e30b17d398ff3b2116364fa.png)

####  指针变量的空格

![img](https://i-blog.csdnimg.cn/blog_migrate/590930e10e5db3d40471443ef0263bd6.png)

 left 左对齐的效果：![img](https://i-blog.csdnimg.cn/blog_migrate/cd4b3cc89cbd6f72ae8bd987eb263595.png)

right右对齐的效果：![img](https://i-blog.csdnimg.cn/blog_migrate/35c83998a0741a226f0e20e294bc25ab.png)

 center居中对齐的效果：![img](https://i-blog.csdnimg.cn/blog_migrate/4a70063507895d5ebc0cf5d35135105a.png)
