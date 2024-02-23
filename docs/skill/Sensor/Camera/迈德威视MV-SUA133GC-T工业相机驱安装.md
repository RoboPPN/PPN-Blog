**1.首先要下载一个驱动文件:**

[网盘链接](https://pan.baidu.com/s/1DESNsyxfp7Fk0ITINlqnjw)  
提取密码: u6wl

**2.下载好后解压文件,进入linuxSDK驱动文件夹，打开终端输入**

```shell
 sudo bash install.sh
```

**3.根据本机系统格式在lib文件夹中选取合适的libMVSDK.so文件**

比如我的系统是arm64的,就要进入到arm64这个文件夹中
在终端输入:

```shell
sudo su

sudo cp libMVSDK.so /lib

reboot
```

**4.驱动安装到此结束**
