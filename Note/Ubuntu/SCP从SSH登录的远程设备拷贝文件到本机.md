## SCP从SSH登录的远程设备拷贝文件到本机

你可以使用`scp`命令从SSH登录的远程设备上拷贝文件到本地。`scp`（Secure Copy Protocol）通过SSH安全传输文件。以下是使用`scp`的基本语法和步骤：

### 基本语法：
```bash
scp [远程用户名]@[远程主机IP]:[远程文件路径] [本地文件路径]
```

### 示例步骤：
假设你希望从远程设备（例如`192.168.1.10`）上将文件`/home/user/remote_file.txt`拷贝到本地的`/home/local_user/`目录：

```bash
scp user@192.168.1.10:/home/user/remote_file.txt /home/local_user/
```

### 参数说明：
- `user@192.168.1.10`：远程设备的用户名和IP地址。
- `/home/user/remote_file.txt`：远程设备上文件的完整路径。
- `/home/local_user/`：本地文件保存的路径。

### 其他选项：
- **拷贝整个文件夹**：使用`-r`选项递归地拷贝整个文件夹。
    ```bash
    scp -r user@192.168.1.10:/home/user/remote_folder /home/local_user/
    ```
- **指定端口**：如果SSH使用了非默认端口（22），可以使用`-P`参数指定端口号。
    ```bash
    scp -P 2222 user@192.168.1.10:/home/user/remote_file.txt /home/local_user/
    ```

执行后，系统会提示你输入远程设备的密码，完成后文件就会被安全地传输到本地。