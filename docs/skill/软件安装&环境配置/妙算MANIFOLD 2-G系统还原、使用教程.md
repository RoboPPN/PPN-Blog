### Manifold2简介 
MANIFOLD 2是DJI为Onboard SDK开发者打造的第二代微型计算机，分为两个版本:
Manifold2-G（128G），Manifold 2-C（256G）。Manifold 2-G（128G）搭载NVIDIA Jetson TX2模块，具备卓越的处理能力和响应速度，扩展灵活，可更加快速地完成复杂的图形处理工作，同时具备Wi-Fi功能，用于网络连接。


[妙算 Manifold 2G使用说明手册、产品信息](https://www.robomaster.com/zh-CN/products/components/detail/2762)


### 安装妙算步骤
在安装妙算前一定要看上面发的妙算说明手册。

1、先将电源适配器接到多功能电源模块,不要接通电源,千万不要接通电源；

2、用XT30电源线将电源模块的1号口连接妙算上的电源接口；

3、I/O线用于将开关控制拓展单元与妙算上的I/O接口连接；

4、将USB拓展单元接到妙算上的USB3.0接口；

到这基本的线路连接就到此结束,后面接上鼠标键盘以及显示器,足以用于妙算的刷机。


### 妙算连接好的实物图

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210316165654443.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)


![在这里插入图片描述](https://img-blog.csdnimg.cn/20210316165812281.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210316165827610.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210316165839759.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)


### 系统还原准备工作
1. 准备一台 Ubuntu 系统的计算机作为主机(如 Manifold 2-C),并确保硬盘空间大于 50 GB。我这里使用的是双系统的Ubuntu18.04系统，给它分了200G内存；
2. 访问以下网址获取最新 DJI 官方镜像文件。此文件包含 DJI 官方镜像以及进行镜像备份和系统还原所需的工具。 [官方镜像](https://www.dji.com/manifold-2/downloads)


### 进入恢复模式
1. 连接开关控制扩展单元至 Manifold 2-G；
2. 连接 Manifold 2-G 的 USB 3.0 接口(Micro-B)至主机；
3. 连接 Manifold 2-G 电源；
4. 按住开关控制扩展单元的 RCV 按键,再按住 RST 按键,2 秒后同时松开两个按键；
5. 在主机的终端界面中输入 `lsusb`,若显示有 `NVIDIA` 设备,则成功进入恢复模式。若未显示 `NVIDIA` 设备,则检查连线及进入方式是否正确,然后重试。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210119162956142.png)

### 系统还原
**使用 DJI 官方镜像**

1、进入恢复模式。

2、在主机的终端界面,进入镜像文件所在目录,然后输入以下命令,以解压官方镜像压缩包。
```shell
sudo tar -zxvf manifold2G_image_V0.4.4.0.tar.gz
```

3、进入解压文件夹下的 Linux_for_Tegra 目录,然后输入以下命令进行还原。

```shell
sudo ./flash.sh jetson-tx2 mmcblk0p1
```

压缩完成时终端显示：

![在这里插入图片描述](https://img-blog.csdnimg.cn/2021011911312175.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)



> 注意：若刷了进不去就把压缩的固件库删除再解压一遍并再刷一遍（重复以上3个步骤），并把主机与妙算2的连接线换成USB 2.0  
> 我重复了3次才成功

下面放一下系统还原成功进去后的图片

cool!
![在这里插入图片描述](https://img-blog.csdnimg.cn/2021011916511232.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)



