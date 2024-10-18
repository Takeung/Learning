## 新到设备连接交换机后主机无法通过WIFI访问

新到设备的默认路由设置中，多数以usb2为最高优先级设备，此时通过WIFI访问内网的主机无法ping通这些新到设备。

解决方法是：

**通过 Netplan 或 NetworkManager 配置默认路由**：

- 如果使用 **Netplan** 管理网络，可以在 `/etc/netplan/` 下的配置文件中明确指定 `eth0` 为默认路由接口，确保以太网的优先级高于 USB。

完成配置的示例如下：

```bash
nvidia@tegra-ubuntu:~$ ll /etc/netplan/
total 28
drwxr-xr-x   2 root   root    4096 Oct 15 14:24 ./
drwxr-xr-x 154 root   root   12288 Oct 12 19:47 ../
-rw-r--r--   1 nvidia nvidia   714 Jul  9 15:57 01-network-manager-4g-5g.yaml-bak
-rw-------   1 root   root     466 Oct 15 14:23 01-network-manager-all.yaml
-rw-------   1 nvidia nvidia   103 Jul 18 15:22 02-wifi-dhcp.yaml

nvidia@tegra-ubuntu:~$ sudo cat /etc/netplan/01-network-manager-all.yaml 
[sudo] password for nvidia: 
# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: NetworkManager

  ethernets:
    eth0:
      dhcp4: no
      addresses: [172.26.1.208/24]
      nameservers:
        addresses: [172.26.253.22,192.168.2.10,114.114.114.114]
      routes:
        - to: default
          via: 172.26.1.1
          metric: 0
        - to: 172.26.0.0/16
          via: 172.26.1.1
          metric: 0

    eth2:
      addresses: [192.168.11.208/24]
```

详细解释如下：

> 该配置文件是 Ubuntu 系统下的 `Netplan` 配置，用于定义网络接口的设置，配置的内容说明如下：
>
> ### 文件头部配置
>
> ```yaml
> network:
>   version: 2
>   renderer: NetworkManager
> ```
>
> - **`network:`**：定义网络配置的根元素。
> - **`version: 2`**：指定使用 Netplan 配置文件的版本。在 Netplan 的 YAML 文件中，`version: 2` 是标准。
> - **`renderer: NetworkManager`**：指定网络管理器的渲染器，这里设置为 `NetworkManager`，通常用于桌面环境或需要使用 `NetworkManager` 作为管理工具的场景。`NetworkManager` 管理网络配置，适合动态切换网络环境的需求。
>
> ### `eth0` 接口配置
>
> ```yaml
> ethernets:
>   eth0:
>     dhcp4: no
>     addresses: [172.26.1.208/24]
>     nameservers:
>       addresses: [172.26.253.22,192.168.2.10,114.114.114.114]
>     routes:
>       - to: default
>         via: 172.26.1.1
>         metric: 0
>       - to: 172.26.0.0/16
>         via: 172.26.1.1
>         metric: 0
> ```
>
> - **`ethernets:`**：定义有线以太网接口配置。
>   
> #### `eth0` 子项
>
> - **`eth0:`**：这是网络接口的名称，表示有线以太网接口 `eth0`，每个物理网卡都会有一个类似的名称，如 `eth0`、`eth1` 等。
>
> - **`dhcp4: no`**：禁用 IPv4 的 DHCP，即 `eth0` 接口不会通过 DHCP 自动获取 IP 地址，而是使用手动配置的静态地址。
>
> - **`addresses: [172.26.1.208/24]`**：定义静态 IP 地址和子网掩码。`172.26.1.208` 是这个接口的 IP 地址，`/24` 表示子网掩码是 `255.255.255.0`，也就是标准的 24 位网络前缀。
>
> - **`nameservers:`**：定义 DNS 服务器。
>   - **`addresses: [172.26.253.22, 192.168.2.10, 114.114.114.114]`**：定义了三个 DNS 服务器的 IP 地址。系统会按顺序使用这些地址进行 DNS 查询：
>     - `172.26.253.22` 是第一个 DNS 服务器；
>     - `192.168.2.10` 是备用的本地 DNS 服务器；
>     - `114.114.114.114` 是公共 DNS 服务器，通常用于互联网访问（这是中国的公共 DNS 地址之一）。
>
> #### `routes` 子项
>
> - **`routes:`**：配置路由表，用于定义网络流量的转发路径。
>
>   1. **`- to: default`**：表示默认路由，即所有未在其他路由中明确规定的网络流量（如互联网访问）都会通过此路由转发。
>      - **`via: 172.26.1.1`**：表示默认路由的下一跳地址是 `172.26.1.1`，这个 IP 是网关（通常是路由器）的地址。
>      - **`metric: 0`**：路由的优先级，`metric` 数字越小，优先级越高。`0` 是最高优先级。
>
>   2. **`- to: 172.26.0.0/16`**：定义了本地局域网的路由，目标网络是 `172.26.0.0/16`，子网掩码为 `255.255.0.0`，覆盖所有 `172.26.x.x` 的网络。
>      - **`via: 172.26.1.1`**：流向该局域网的流量将通过 `172.26.1.1` 路由器。
>      - **`metric: 0`**：优先级最高。
>
> ### `eth2` 接口配置
>
> ```yaml
> eth2:
>   addresses: [192.168.11.208/24]
> ```
>
> - **`eth2:`**：另一个有线网络接口，名称为 `eth2`。
> - **`addresses: [192.168.11.208/24]`**：为该接口配置了静态 IP 地址 `192.168.11.208`，子网掩码为 `255.255.255.0`（即 `/24`）。
>
> 这段配置没有指定 `eth2` 的网关或 DNS 服务器，意味着 `eth2` 接口用于特定网络（`192.168.11.x` 网络）内的通信，而不是用于互联网或其他网络访问。
>
> ### 总结
>
> - `eth0` 接口被配置为使用静态 IP 地址 `172.26.1.208`，并通过默认路由 `172.26.1.1` 访问外部网络，同时它还具有局域网 `172.26.0.0/16` 的路由配置，允许访问该局域网内的所有设备。
> - `eth2` 接口配置为 `192.168.11.208`，用于 `192.168.11.x` 子网的通信。
> - `nameservers` 定义了多个 DNS 服务器，用于网络名称解析。