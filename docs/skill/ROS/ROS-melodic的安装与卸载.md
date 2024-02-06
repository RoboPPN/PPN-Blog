## 安装ROS
### 1. 添加软件源

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 2. 添加密匙
 
```shell
 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
### 3. 安装ROS
```shell
 sudo apt update
sudo apt install ros-melodic-desktop-full
```
### 4.初始化rosdep

```shell
sudo rosdep init
rosdep update
```
### 5.设置环境变量

```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### 6.安装rosinstall

```shell
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```


### 7.以上步骤完成后我们来启动小乌龟节点试试：

```shell
roscore

rosrun turtlesim turtlesim_node

rosrun turtlesim turtle_teleop_key
```

在这可以看到小乌龟节点顺利启动运行，我们的ROS系统也就安装好了
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210115120258454.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)

## 卸载ROS
### 卸载全部ROS：
1.终端输入：
```shell
sudo apt-get remove --purge ros-* 
```
卸载某个ros版本(ros版本可以共存，每次需要切换)
如melodic:
```shell
sudo apt-get remove --purge ros-melodic
```
或者先卸载包

```shell
sudo apt-get purge ros-*　　 
```
然后删除依赖，配置

```shell
sudo apt-get autoremove
```
2.检查 ~/.bashrc　以及／opt/目录是否有ros文件夹存在，有，则删除


### 另外一种方法
终端输入：

```shell
sudo apt-get purge ros-*
sudo rm -rf /etc/ros
gedit ~/.bashrc
```
找到带有melodic的那一行`删除`，`保存`

终端输入：

```shell
source ~/.bashrc
```