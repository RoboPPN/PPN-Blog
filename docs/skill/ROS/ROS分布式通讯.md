ROS是一种分布式软件框架，节点之间通过松耦合的方式进行组合，在很多应用场景下，节点可以运行在不同的计算平台上，通过Topic、Service进行通信。但是ROS中只允许存在一个Master，在多机系统中Master只能运行在一台机器上，其他机器需要通过ssh的方式和Master取得联系,所以在多机ROS系统中需要进行一些配置。

### 查看主、从机的用户名
```shell
hostname
```

### 查看主、从机的ip
```shell
ifconfig
```


### 主机端.bashrc文件做如下修改


在最后几行加入：

```sh
export ROS_HOSTNAME=192.168.1.135（主机的IP）
export ROS_MASTER_URI=http://192.168.1.135:11311  	（主机的ip地址）
export ROS_IP=192.168.1.146  （从机的ip地址）
```

### 从机端.bashrc文件做如下修改
```sh
export ROS_HOSTNAME=192.168.1.146（从机的IP）
export ROS_MASTER_URI=http://192.168.1.135:11311  	（主机的ip地址）
export ROS_IP=192.168.1.146  （从机的ip地址）
```

### 执行
1. 在主机终端输入：
```shell
roscore
```

2. 如需要检测是否连接成功，需要在从机终端输入：
```shell
rostopic list
```

**能看到话题就代表连接成功了**

### 在自己电脑终端打开别人的终端
首先需要安装`ssh`
```shell
sudo apt-get install ssh
```

安装完成后在终端输入一下内容：
```shell
ssh hostname(另一台电脑的用户名)@(另一台电脑的IP地址)
```

例如：
```shell
ssh zdg@192.168.1.1
```

之后输入另一台电脑的登录密码就可以在自己的电脑中打开他人的终端了



