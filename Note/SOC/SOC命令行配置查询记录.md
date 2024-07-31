# SOC命令行配置查询记录

## 配置网络IP

查看网络配置发现ip被写死

```bash
nvidia@nvidia-desktop:~$ ifconfig
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ether 02:42:eb:15:5c:e8  txqueuelen 0  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.55.200  netmask 255.255.255.0  broadcast 192.168.55.255
        inet6 fe80::4ab0:2dff:fe94:3aa5  prefixlen 64  scopeid 0x20<link>
        ether 48:b0:2d:94:3a:a5  txqueuelen 1000  (Ethernet)
        RX packets 958  bytes 237495 (237.4 KB)
        RX errors 0  dropped 3  overruns 0  frame 0
        TX packets 48  bytes 5309 (5.3 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
        device memory 0x2b28000000-2b2807ffff

eth1: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1466
        ether 48:b0:2d:94:3a:a3  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

eth2: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1466
        inet6 fe80::4ab0:2dff:fe94:3aab  prefixlen 64  scopeid 0x20<link>
        ether 48:b0:2d:94:3a:ab  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 21  bytes 4095 (4.0 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 224  bytes 15208 (15.2 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 224  bytes 15208 (15.2 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

usb0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 4a:ac:2b:17:70:9d  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

usb1: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 4a:ac:2b:17:70:9f  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

usb2: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.225.20  netmask 255.255.255.0  broadcast 192.168.225.255
        inet6 fe80::c054:7ff:fe3e:10a0  prefixlen 64  scopeid 0x20<link>
        ether c2:54:07:3e:10:a0  txqueuelen 1000  (Ethernet)
        RX packets 113  bytes 8536 (8.5 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 123  bytes 15738 (15.7 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

进入netplan配置文件确认

```bash
nvidia@nvidia-desktop:~$ sudo cat /etc/netplan/01-network-manager-all.yaml
[sudo] password for nvidia:
# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
      dhcp6: true
      addresses: [192.168.55.200/24]
    eth2:
      dhcp4: true
      dhcp6: true
        #addresses: [192.168.55.100/24]
```

可以先注释掉强制设定IP的代码

```bash
nvidia@nvidia-desktop:~$ sudo vi /etc/netplan/01-network-manager-all.yaml
# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: networkd
  	#ethernets:
    #    eth0:
    #  dhcp4: true
    #  dhcp6: true
    #  addresses: [192.168.55.200/24]
    #eth2:
    #  dhcp4: true
    #  dhcp6: true
    #    #addresses: [192.168.55.100/24]
```

应用网络配置

```bash
nvidia@nvidia-desktop:~$ sudo netplan apply

** (generate:2629): WARNING **: 10:38:09.358: Permissions for /etc/netplan/01-network-manager-4g-5g.yaml are too open. Netplan configuration should NOT be accessible by others.
WARNING:root:Cannot call Open vSwitch: ovsdb-server.service is not running.

** (process:2627): WARNING **: 10:38:09.893: Permissions for /etc/netplan/01-network-manager-4g-5g.yaml are too open. Netplan configuration should NOT be accessible by others.

** (process:2627): WARNING **: 10:38:10.557: Permissions for /etc/netplan/01-network-manager-4g-5g.yaml are too open. Netplan configuration should NOT be accessible by others.

** (process:2627): WARNING **: 10:38:10.557: Permissions for /etc/netplan/01-network-manager-4g-5g.yaml are too open. Netplan configuration should NOT be accessible by others.
```

重启设备后查看网络自动分配的IP

```bash
nvidia@nvidia-desktop:~$ ifconfig
docker0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 172.17.0.1  netmask 255.255.0.0  broadcast 172.17.255.255
        ether 02:42:b1:ec:23:e8  txqueuelen 0  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 172.26.1.163  netmask 255.255.255.0  broadcast 172.26.1.255
        inet6 fe80::fb5b:1b0f:951e:4f49  prefixlen 64  scopeid 0x20<link>
        ether 48:b0:2d:94:3a:a5  txqueuelen 1000  (Ethernet)
        RX packets 108  bytes 41656 (41.6 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 59  bytes 23096 (23.0 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
        device memory 0x2b28000000-2b2807ffff

eth1: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1466
        ether 48:b0:2d:94:3a:a3  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

eth2: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1466
        inet6 fe80::8200:eaab:4473:2372  prefixlen 64  scopeid 0x20<link>
        ether 48:b0:2d:94:3a:ab  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 15  bytes 2817 (2.8 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 159  bytes 12340 (12.3 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 159  bytes 12340 (12.3 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

usb0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 4a:ac:2b:17:70:9d  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

usb1: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 4a:ac:2b:17:70:9f  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

usb2: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.225.46  netmask 255.255.255.0  broadcast 192.168.225.255
        inet6 fe80::bccc:e0ff:fe10:f7a6  prefixlen 64  scopeid 0x20<link>
        ether be:cc:e0:10:f7:a6  txqueuelen 1000  (Ethernet)
        RX packets 34  bytes 2860 (2.8 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 31  bytes 4973 (4.9 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

将自动分配的IP重新写入到netplan的配置中并应用

```bash
nvidia@nvidia-desktop:~$ sudo vi /etc/netplan/01-network-manager-all.yaml
# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: networkd
    ethernets:
      eth0:
        dhcp4: true
        dhcp6: true
        addresses: [172.26.1.163/24]
      eth2:
        dhcp4: true
        dhcp6: true
        #addresses: [192.168.55.100/24]
```

推荐格式如下

```bash
eth0:
    dhcp4: no
    addresses: [172.26.1.137/24]
    nameservers:
      addresses: [114.114.114.114,192.168.2.10]
    routes:
    - to: default
      via: 172.26.1.1
      metric: 0
    - to: 172.26.0.0/16
      via: 172.26.1.1
      metric: 0
```

> 这段配置是用于设置网络接口 `eth0` 的网络参数。具体作用如下：
>
> 1. **禁用 DHCP**：
>    ```yaml
>    dhcp4: no
>    ```
>    这行表示禁用 `eth0` 接口的 DHCP（动态主机配置协议），即不会自动从 DHCP 服务器获取 IP 地址和其他网络配置。
>
> 2. **静态 IP 地址**：
>    ```yaml
>    addresses: [172.26.1.137/24]
>    ```
>    为 `eth0` 接口分配静态 IP 地址 `172.26.1.137`，子网掩码为 `255.255.255.0`。
>
> 3. **DNS 服务器**：
>    ```yaml
>    nameservers:
>      addresses: [114.114.114.114,192.168.2.10]
>    ```
>    配置 DNS 服务器地址为 `114.114.114.114` 和 `192.168.2.10`。
>
> 4. **路由配置**：
>    ```yaml
>    routes:
>    - to: default
>      via: 172.26.1.1
>      metric: 0
>    - to: 172.26.0.0/16
>      via: 172.26.1.1
>      metric: 0
>    ```
>    - **默认路由**：
>      ```yaml
>      - to: default
>        via: 172.26.1.1
>        metric: 0
>      ```
>      设置默认网关为 `172.26.1.1`，所有未明确指定路由的流量将通过这个网关转发。
>    - **特定子网路由**：
>      ```yaml
>      - to: 172.26.0.0/16
>        via: 172.26.1.1
>        metric: 0
>      ```
>      设置到 `172.26.0.0/16` 网段的流量通过网关 `172.26.1.1` 进行转发。
>
> 总的来说，这段配置为 `eth0` 接口设置了一个静态 IP 地址，指定了 DNS 服务器，以及配置了默认网关和一个特定子网的静态路由。

## 存储信息查询

查看硬盘存量

```bash
sudo fdisk -l
```

```bash
Disk /dev/loop0: 4 KiB, 4096 bytes, 8 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes


Disk /dev/loop1: 69.22 MiB, 72581120 bytes, 141760 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes


Disk /dev/loop2: 245.48 MiB, 257404928 bytes, 502744 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes


Disk /dev/loop3: 245.73 MiB, 257662976 bytes, 503248 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes


Disk /dev/loop4: 483.33 MiB, 506810368 bytes, 989864 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes


Disk /dev/loop5: 91.69 MiB, 96141312 bytes, 187776 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes


Disk /dev/loop6: 33.71 MiB, 35344384 bytes, 69032 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes


Disk /dev/loop7: 16 MiB, 16777216 bytes, 32768 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0x00000000


Disk /dev/mmcblk0: 59.28 GiB, 63652757504 bytes, 124321792 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: gpt
Disk identifier: 07E65CA6-0339-48DD-AC1B-B55D097D0A04

Device            Start       End   Sectors   Size Type
/dev/mmcblk0p1  3050048 124321751 121271704  57.8G Microsoft basic data
/dev/mmcblk0p2       40    262183    262144   128M Microsoft basic data
/dev/mmcblk0p3   262184    263719      1536   768K Microsoft basic data
/dev/mmcblk0p4   263720    328487     64768  31.6M Microsoft basic data
/dev/mmcblk0p5   328488    590631    262144   128M Microsoft basic data
/dev/mmcblk0p6   590632    592167      1536   768K Microsoft basic data
/dev/mmcblk0p7   592168    656935     64768  31.6M Microsoft basic data
/dev/mmcblk0p8   656936    820775    163840    80M Microsoft basic data
/dev/mmcblk0p9   820776    821799      1024   512K Microsoft basic data
/dev/mmcblk0p10  821800    952871    131072    64M EFI System
/dev/mmcblk0p11  952872   1116711    163840    80M Microsoft basic data
/dev/mmcblk0p12 1116712   1117735      1024   512K Microsoft basic data
/dev/mmcblk0p13 1117736   1248807    131072    64M Microsoft basic data
/dev/mmcblk0p14 1248832   2068031    819200   400M Microsoft basic data
/dev/mmcblk0p15 2068032   3050047    982016 479.5M Microsoft basic data

Partition table entries are not in disk order.


Disk /dev/nvme0n1: 223.57 GiB, 240057409536 bytes, 468862128 sectors
Disk model: M.2 (P80) 3TE4
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: gpt
Disk identifier: 3A05008A-186C-4ED6-AC82-40517F575613

Device              Start       End   Sectors   Size Type
/dev/nvme0n1p1    3050048 170822207 167772160    80G Microsoft basic data
/dev/nvme0n1p2  170822208 338594367 167772160    80G Microsoft basic data
/dev/nvme0n1p3         40    262183    262144   128M Microsoft basic data
/dev/nvme0n1p4     262184    263719      1536   768K Microsoft basic data
/dev/nvme0n1p5     263720    328487     64768  31.6M Microsoft basic data
/dev/nvme0n1p6     328488    590631    262144   128M Microsoft basic data
/dev/nvme0n1p7     590632    592167      1536   768K Microsoft basic data
/dev/nvme0n1p8     592168    656935     64768  31.6M Microsoft basic data
/dev/nvme0n1p9     656936    820775    163840    80M Microsoft basic data
/dev/nvme0n1p10    820776    821799      1024   512K Microsoft basic data
/dev/nvme0n1p11    821800    952871    131072    64M EFI System
/dev/nvme0n1p12    952872   1116711    163840    80M Microsoft basic data
/dev/nvme0n1p13   1116712   1117735      1024   512K Microsoft basic data
/dev/nvme0n1p14   1117736   1248807    131072    64M Microsoft basic data
/dev/nvme0n1p15   1248832   2068031    819200   400M Microsoft basic data
/dev/nvme0n1p16   2068032   3050047    982016 479.5M Microsoft basic data

Partition table entries are not in disk order.


Disk /dev/zram0: 1.87 GiB, 2011873280 bytes, 491180 sectors
Units: sectors of 1 * 4096 = 4096 bytes
Sector size (logical/physical): 4096 bytes / 4096 bytes
I/O size (minimum/optimal): 4096 bytes / 4096 bytes


Disk /dev/zram1: 1.87 GiB, 2011873280 bytes, 491180 sectors
Units: sectors of 1 * 4096 = 4096 bytes
Sector size (logical/physical): 4096 bytes / 4096 bytes
I/O size (minimum/optimal): 4096 bytes / 4096 bytes


Disk /dev/zram2: 1.87 GiB, 2011873280 bytes, 491180 sectors
Units: sectors of 1 * 4096 = 4096 bytes
Sector size (logical/physical): 4096 bytes / 4096 bytes
I/O size (minimum/optimal): 4096 bytes / 4096 bytes


Disk /dev/zram3: 1.87 GiB, 2011873280 bytes, 491180 sectors
Units: sectors of 1 * 4096 = 4096 bytes
Sector size (logical/physical): 4096 bytes / 4096 bytes
I/O size (minimum/optimal): 4096 bytes / 4096 bytes


Disk /dev/zram4: 1.87 GiB, 2011873280 bytes, 491180 sectors
Units: sectors of 1 * 4096 = 4096 bytes
Sector size (logical/physical): 4096 bytes / 4096 bytes
I/O size (minimum/optimal): 4096 bytes / 4096 bytes


Disk /dev/zram5: 1.87 GiB, 2011873280 bytes, 491180 sectors
Units: sectors of 1 * 4096 = 4096 bytes
Sector size (logical/physical): 4096 bytes / 4096 bytes
I/O size (minimum/optimal): 4096 bytes / 4096 bytes


Disk /dev/zram6: 1.87 GiB, 2011873280 bytes, 491180 sectors
Units: sectors of 1 * 4096 = 4096 bytes
Sector size (logical/physical): 4096 bytes / 4096 bytes
I/O size (minimum/optimal): 4096 bytes / 4096 bytes


Disk /dev/zram7: 1.87 GiB, 2011873280 bytes, 491180 sectors
Units: sectors of 1 * 4096 = 4096 bytes
Sector size (logical/physical): 4096 bytes / 4096 bytes
I/O size (minimum/optimal): 4096 bytes / 4096 bytes
```

另一条指令

```bash
df -h
```

```bash
Filesystem      Size  Used Avail Use% Mounted on
/dev/nvme0n1p1   79G   28G   48G  37% /
tmpfs            15G  136K   15G   1% /dev/shm
tmpfs           6.0G   19M  6.0G   1% /run
tmpfs           5.0M  4.0K  5.0M   1% /run/lock
tmpfs           3.0G   96K  3.0G   1% /run/user/128
tmpfs           3.0G   84K  3.0G   1% /run/user/1000
```