## cartographer的安装与编译

点击此处->[编译 Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)

编译需执行以下语句：

```shell
catkin_make_isolated --install --use-ninja
```

之前一直使用的`catkin_make`是用于编译工作空间下所有功能包（要求各个功能包不重复），在安装 cartographer 时，它有重复的文件名，这就导致了 catkin_make 失效。所以在使用 cartographer 框架时，应使用 `catkin_make_isolated --install --use-ninja`来对各个功能包进行单独编译。

## 设置环境变量

cartographer 里面没有编译生成的 devel 文件夹，但是有 install_isolated 。

所以对于 cartographer，我们可以这样给它设置环境变量：

```shell
source install_isolated/setup.bash 
```

### 为什么要设置环境变量？

运行setup文件可以使系统使用这个工作区以及其中包含的代码。

## 使用 cartographer 进行建图

1. 把运行激光雷达的功能包放入到工作空间的 src 文件下下，运行编译指令；
2. 测试雷达是否能正常运行，正常运行则跳步骤3；
3. 利用 cartographer 进行建图；

   需要修改两个文件： revo_lds.lua 和 emo_revo_lds.launch

   3.1 修改revo_lds.lua文件

    ```shell
    sudo gedit ~/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files/revo_lds.lua
    ```

    复制原先的文件，新建一个my_revo_lds.lua文件并修改：

    ```shell
    tracking_frame = "horizontal_laser_link",   //SLAM算法跟踪坐标系的名称

    published_frame = "horizontal_laser_link"  //用作发布位姿子坐标系的名称
    ```

    修改为：

    ```shell
    tracking_frame = "laser_link",   //SLAM算法跟踪坐标系的名称

    published_frame = "laser_link"  //用作发布位姿子坐标系的名称
    ```

   3.2修改demo_revo_lds.launch

    ```shell
    sudo gedit ~/catkin_ws/src/cartographer_ros/cartographer_ros/launch/demo_revo_lds.launch
    ```

    复制原先的文件，新建为my_demo_revo_lds.launch直接把原先的文件内容改成下面这个

    因为非bag仿真,将以下true改为false

    ```html
    <param name="/use_sim_time" value="true" />
    ```

    使用我们的配置文件,将revo_lds.lua改为my_revo_lds.lua

    ```html
    <node name="cartographer_node" pkg="cartographer_ros"

      type="cartographer_node" args="

          -configuration_directory $(find cartographer_ros)/configuration_files

          -configuration_basename revo_lds.lua"
    ```

    将horizontal_laser_2d改为我们的激光雷达输出话题scan_XX

    ```html
    <remap from="scan" to="horizontal_laser_2d" />
    ```

    不使用bag,删去以下内容

    ```html
    <node name="playbag" pkg="rosbag" type="play"

      args="--clock $(arg bag_filename)" />
    ```

    注：每次修改完配置都需要进行编译。

    ```html
    catkin_make_isolated --install --use-ninja 
    ```

4. 运行cartographer框架

    ```shell
    roslaunch cartographer_ros my_demo_revo_lds.launch
    ```

## 参考致谢

- [ROS学习-cartographer安装和使用](https://blog.csdn.net/popvan/article/details/123354259)
