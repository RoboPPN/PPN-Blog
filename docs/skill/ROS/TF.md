## 一.tf摘要
tf是一个软件包,可让用户随时间跟踪多个坐标系.tf保持时间缓冲树结构中的坐标系之间的关系,并允许用户在任意所需的时间点在任意两个坐标系之间的转换点,向量等
## 二.tf简介演示
描述:该演示能更好的理解tf能做什么事
1.下载demo

```cpp
sudo apt-get install ros-melodic-ros-tutorials ros-melodic-geometry-tutorials ros-melodic-rviz ros-melodic-rosbash ros-melodic-rqt-tf-tree
```
2.运行demo

```cpp
roslaunch turtle_tf turtle_tf_demo.launch
```
### tf命令行工具的使用
#### 1.view_frame
view_frame的作用是创建tf通过ROS广播帧的示意图

```cpp
rosrun tf view_frames
```
在终端运行上面的命令行会出现:

```cpp
Listening to /tf for 5.0 seconds
Done Listening
dot - graphviz version 2.40.1 (20161225.0304)

Detected dot version 2.40
frames.pdf generated
```
上里是tf侦听器正在侦听通过ROS广播的帧,并绘制连接方式的树(绘制成功的树的pdf会自动存储在终端路径下面).
#### 2.evince frames.pdf
evince frames.pdf的作用是查看树
```cpp
evince frames.pdf
```
上面这行指令的作用就是打开绘制好好的PDF文件
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210321143937696.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)

> 注意:如果把绘制好的PDF文件删除后,运行`evince frames.pdf`时就会报错

该演示使用tf库创建了三个坐标框架:world框架,turtle1框架和turtle2框架
在这里我们可以看到tf广播的三个帧:world,turtle1,turtle2.还可以看到world是turtle1和turtle2框架的父代.
出于调试目的,view_frame还报告一些诊断信息,这些信息有关何时接收到最旧和最新的帧转换以及tf帧发布到tf的速度.



#### 3.rqt_tf_tree
rqt_tf_tree是一个实时运行工具,用于可视化ROS广播的帧树,仅可通过图左上角的刷新来刷新树.

```cpp
rosrun rqt_tf_tree rqt_tf_tree
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210321152003224.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)
#### 4.rqt＆
rqt＆是打开一个集中许多rqt工具的工具箱
运行:

```cpp
rqt＆
```
在`Plugins`中选择你想要的工具
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210321152746704.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70)

#### 5.tf_echo
tf_echo的作用是报告ROS广播的任何两个帧之间的转换

```cpp
rosrun tf tf_echo [reference_frame] [target_frame]
```
turtle2框架相对于turtle1框架的变换:

```cpp
rosrun tf tf_echo turtle1 turtle2
```

#### 6.tf_monitor
tf_monitor的作用是将所有坐标变换树的信息输出到控制台

```cpp
rosrun tf tf_monitor
```

```cpp
RESULTS: for all Frames

Frames:

All Broadcasters:

RESULTS: for all Frames

Frames:
Frame: turtle1 published by unknown_publisher Average Delay: 0.000291761 Max Delay: 0.000337576
Frame: turtle2 published by unknown_publisher Average Delay: 0.000279885 Max Delay: 0.000339924

All Broadcasters:
Node: unknown_publisher 135.709 Hz, Average Delay: 0.000285823 Max Delay: 0.000339924
```

```handlebars
tf_monitor <源框架> <目标目标>
```
监视特定的转换。例如，监视从/ base_footprint到/ odom的转换：

```handlebars
rosrun tf tf_monitor /base_footprint /odom
```

#### 7.roswtf插件
roswtf TF附带了一个插件roswtf每次运行时自动运行roswtf.该插件将分析你当前的tf配置,并尝试查找常见问题.要运行,只需正常调用roswtf即可:

```handlebars
roswtf
```
## 三.教程讲解
### 在TF中使用传感器信息
#### 在`tf::MessageFilter`中使用`Stamped`数据类型
如何使用tf::MessageFilter处理Stamped数据类型???
本教程介绍了如何在tf中使用传感器数据.
传感器:例如相机,激光雷达等

> 假设我们创建了一个新的乌龟,叫turtle3,并且没有里程表,但是有一个高速摄像机跟踪turtle3的位置并将位置发布为与世界框架有关的geometry_msgs/PointStamped信息

> turtle1想要知道turtle3与自己的坐标关系,为此,turtle1必须聆听正在发布turtle3姿势的主题,等到准备好转换为所需的帧后再进行操作

> 为了简化操作,tf::MessageFilter类非常有用.tf::MessageFilter将使用标头订阅任何ros消息并对其进行缓存,直到可以将其转换为目标帧为止


```cpp
roslaunch turtle_tf turtle_tf_sensor.launch 
```
运行上面的命令行将使turtle1自行驱动,turtle3自行移动.有一个主题turtle_pose_stamped,其中包含`geometry_msgs / PoseStamped`数据,说明了turtle3在世界框架中的位置.
为了可靠的在turtle1的帧中获取数据流,我们将使用以下代码:

```cpp
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

class PoseDrawer
{
public:
  PoseDrawer() : tf_(),  target_frame_("turtle1")
  {
    point_sub_.subscribe(n_, "turtle_point_stamped", 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PointStamped>(point_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
  } ;

private:
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<geometry_msgs::PointStamped> * tf_filter_;
  ros::NodeHandle n_;
  std::string target_frame_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr) 
  {
    geometry_msgs::PointStamped point_out;
    try 
    {
      tf_.transformPoint(target_frame_, *point_ptr, point_out);
      
      printf("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
             point_out.point.x,
             point_out.point.y,
             point_out.point.z);
    }
    catch (tf::TransformException &ex) 
    {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
  };

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pose_drawer"); //Init ROS
  PoseDrawer pd; //Construct class
  ros::spin(); // Run until interupted 
};
```
[代码详细解析](http://wiki.ros.org/tf/Tutorials/Using%20Stamped%20datatypes%20with%20tf::MessageFilter)





### 使用tf设置机器人
#### 1.Transform Configuration(转换配置)

> 许多ROS软件包要求使用tf发布机器人的变换树软件库.变换树根据不同的坐标系之间的平移和旋转来定义偏移量.
为了更好的了解变换树这个概念,接下来请看一个简单的机器人示例:

>机器人具有一个可移动的基座,并在其上方安装了一个激光雷达,在提及机器人时,我们定义两个坐标系:一个对应机器人基座的中心点,另外一个对应安装在基座顶部的激光雷达的中心点

>我们给这两个坐标系起个名字以便于参考.对于框架命名约定,我们将附着在移动基座上的坐标系称为"base_link"(对于导航来说,将坐标系放置在机器人的旋转中心非常重要),将附着在激光雷达上的坐标系称为"base_laser"

>假设我们有一些来自激光雷达的数据,其形式是到激光雷达中心点的距离。换句话说,我们在"base_laser"坐标系中有一些数据

> 现在假设我们要获取这些数据并将其用于帮助机器人避障.为了成功完成避障任务，我们需要一种将"base_laser"接受到的激光扫描信息转换为"base_link"帧的方法

**总结：我们需要定义"base_laser"和"base_link"坐标系之间的关系。**


![在这里插入图片描述](https://img-blog.csdnimg.cn/20210322143850715.png)

>在定义这种关系时，假设我们知道激光雷达安装在向前10厘米、可移动基座中心点上方20厘米的位置。这提供了一个平移偏移量，该偏移量将"base_link"框架与"base_laser"框架相关联

>具体来说，我们知道，要从"base_link"框架获取数据到"base_laser"框架，我们必须应用（x:0.1m,y:0.0m,z:0.2m)的转换，并且从"base_laser"帧到"base_link"帧，就要用相反的平移(x:-0.1m,y:0.0m,z:-0.2m)

>我们可以自己选择自己管理这种关系，意味着在必要时在框架之间存储和应用适当的转换，但是随着坐标框架数量的增加，工作量和复杂程度会呈指数增长。幸运的是，我们不必自己做这项工作。我们可以使用tf一次定义"base_link"和"base_laser"之间的关系，并且让它为我们管理两个坐标系之间的转换关系

>要使用tf定义和存储“ base_link”和“ base_laser”帧之间的关系，我们需要将它们添加到转换树中。从概念上讲，变换树中的每个节点都对应于一个坐标框架，每个边缘都对应于需要应用以从当前节点移动到其子节点的变换。Tf使用树结构来确保只有一个遍历将任意两个坐标系链接在一起，并假定树中的所有边都从父节点指向子节点

![在这里插入图片描述](https://img-blog.csdnimg.cn/2021032214391950.png)

>我们简单示例创建一个转换树，我们将创建两个节点，一个节点用于“ base_link”坐标框架，而一个节点用于“ base_laser”坐标框架。为了在它们之间创建边缘，我们**首先需要确定哪个节点将是父节点，哪个节点将是子节点**。请记住，这种区别很重要，因为**tf假定所有转换都从父级移动到子级**。让我们选择“ base_link”坐标系作为父坐标系，因为随着其他零件/传感器被添加到机器人中，通过遍历“ base_link”框架将它们与“ base_laser”坐标系联系起来是最有意义的。这意味着与连接“ base_link”和“ base_laser”的边相关的变换应为（x：0.1m，y：0.0m，z：0.2m）。设置好此转换树后，将在“ base_laser”帧中接收的激光扫描转换为“ base_link”帧就像调用tf库一样简单。机器人可以使用此信息来推断“ base_link”框架中的激光扫描，并安全地规划周围环境中的障碍物



#### 2.编写代码

> 现在，我们用代码来创建转换树。 我们要完成上面描述的任务：在"base_laser"帧中获取点并将其转换为"base_link"帧。
> 我们需要做的第一件事情就是创建一个节点，该节点将负责在我们的系统中发布转换数据。接下来，我们将必须创建一个节点来侦听通过ROS发布的转换数据，并将其应用于变换点

创建一个官方源码包，命名为"robot_setup_tf "，添加roscpp tf geometry_msgs依赖
```cpp
cd catkin_ws/src
catkin_create_pkg robot_setup_tf roscpp tf geometry_msgs
```
安装navigation-tutorials 包

```cpp
sudo apt-get install ros-melodic-navigation-tutorials 
```
#### 3.Broadcasting a Transform
任务：创建一个节点来完成通过ROS广播base_laser---->base_link转换工作，在src文件夹下创建tf_broadcaster.cpp文件，放入以下内容：

```cpp
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  }
}
```

有关代码解释:[http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF](http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF)


#### 4.Using a Transform
>上面，我们创建了一个节点，该节点通过ROS发布base_laser → base_link转换

任务:编写一个节点，该节点将使用该变换获取“ base_laser”框架中的一个点并将其变换为“ base_link”框架中的一个点。在src文件夹下创建tf_listener.cpp文件，放入以下内容：


```cpp
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
```

有关代码解释:http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF



### 通过ROS发布传感器流
描述:本教程提供了通过ROS发送两种类型的传感器的示例,即sensor_mags/LaserScan消息和sensor_msgs/PointCloud消息

#### 1.通过ROS发布传感器流
通过ROS正确的从传感器发布数据对于导航堆栈的安全运行非常重要.如果导航堆栈没有从机器人的传感器接收到任何信息，那么它将导致盲目行驶，很可能会撞到东西。有许多传感器可用于向导航堆栈提供信息：激光，摄像机，声纳，红外，碰撞传感器等。但是，当前的导航堆栈仅接受使用sensor_msgs / LaserScan消息类型或sensor_msgs / PointCloud消息类型

#### 2.通过ROS发布LaserScans
##### 2.1   LaserScan信息
对于具有激光扫描仪的机器人，ROS在sensor_msgs软件包中提供了一种称为LaserScan的特殊消息类型，用于保存有关给定扫描的信息。只要可以对从扫描仪返回的数据进行格式化以使其适合消息，LaserScan消息就可以使代码几乎可以轻松地与几乎所有激光器一起使用。在讨论如何生成和发布这些消息之前，让我们看一下消息规范本身：

```cpp
＃
＃激光扫描角度是逆时针测量的，0朝前
设备框架的＃（沿x轴）
＃

Header header
float32 angle_min					＃扫描的起始角度[rad]
float32 angle_max					＃扫描的结束角度[rad]
float32 angle_increment		＃测量之间的角度距离[rad]
float32 time_increment		＃测量之间的时间[秒]
float32 scan_time					＃两次扫描之间的时间[秒]
float32 range_min					＃最小范围值[m]
float32 range_max					＃最大范围值[m]
float32 [] ranges						＃范围数据[m]（注意：值<range_min或> range_max应该被丢弃）
float32 []intensities 				＃强度数据[特定于设备的单位]
```


##### 2.2编写代码以发布LaserScan消息
为了使事情具体，让我们编写一个简单的激光扫描发布器来演示事情的工作原理

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = count;
      intensities[i] = 100 + count;
    }
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    ++count;
    r.sleep();
  }
}
```

代码详解:[http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors)

#### 3.通过ROS发布PointClouds
##### 3.1PointCloud消息
为了存储和共享有关世界上许多点的数据，ROS提供了sensor_msgs / PointCloud消息。如下所示，此消息旨在支持三维维度的点数组以及作为通道存储的任何关联数据。例如，可以通过“intensity”通道通过电线发送PointCloud，该通道保存有关云中每个点的强度值的信息

```cpp
Header header
geometry_msgs/Point32[] points  #3d点数组
ChannelFloat32[] channels       #每个通道应具有与点数组相同的元素数，并且每个通道中的数据应与每个点1：1对应
```

##### 3.2编写代码以发布PointCloud消息

```cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);

  unsigned int num_points = 100;

  int count = 0;
  ros::Rate r(1.0);
  while(n.ok()){
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sensor_frame";

    cloud.points.resize(num_points);

    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    //generate some fake data for our point cloud
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 1 + count;
      cloud.points[i].y = 2 + count;
      cloud.points[i].z = 3 + count;
      cloud.channels[0].values[i] = 100 + count;
    }

    cloud_pub.publish(cloud);
    ++count;
    r.sleep();
  }
}
```
代码详解:[http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors)


### 在自己的机器人上使用机器人状态发布器
描述：本教程介绍了如何使用机器人状态发布器将机器人状态发布到tf

> 当您使用具有许多相关框架的机器人时，将它们全部发布到tf成为一项艰巨的任务。机器人状态发布者是一款可以为您完成此工作的工具

> 机器人状态发布者可帮助您将机器人的状态广播到tf转换库。机器人状态发布者在内部具有机器人的运动学模型。因此，根据机器人的关节位置，机器人状态发布者可以计算并广播机器人中每个链接的3D姿态

详情:[Using the robot state publisher on your own robot](http://wiki.ros.org/robot_state_publisher/Tutorials/Using%20the%20robot%20state%20publisher%20on%20your%20own%20robot)


### 将urdf与robot_state_publisher一起使用

> 描述：本教程提供了使用URDF的机器人模型的完整示例，该模型使用robot_state_publisher。首先，我们创建包含所有必要部分的URDF模型。然后，我们编写一个节点，该节点发布JointState并进行转换


详情:[Using urdf with robot_state_publisher](http://wiki.ros.org/urdf/Tutorials/Using%20urdf%20with%20robot_state_publisher)


任何用户基本都将使用tf进行两项任务,即广播转换监听转换

任何使用tf的人都需要听转换:

- [广播转换](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C++%29)-将坐标系的相对姿势发送到系统的其余部分。一个系统可以有许多广播员，每个广播员都提供有关机器人不同部分的信息
 - [侦听变换](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C++%29)-接收和缓冲系统中广播的所有坐标帧，并查询帧之间的特定变换
 

