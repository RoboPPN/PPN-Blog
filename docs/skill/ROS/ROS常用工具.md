### rqt
运行rqt
```shell
rqt
```
进入上方菜单栏的Plugins，里面有很多十分有用的用于ROS调试的插件


### rqt_tf_tree
`rqt_tf_tree`提供了一个用于可视化 ROS TF 框架树的 GUI 插件

在终端输入
```shell
rosrun rqt_tf_tree rqt_tf_tree 
```
便能在新打开的UI界面查看TF框架树

### rqt_graph

![在这里插入图片描述](https://img-blog.csdnimg.cn/20201117001315233.png#pic_center)

在ros里面，有一系列以rqt开头的工具，他们都是基于qt的可视化工具，利用他们我们可以得到一些很直观的信息。
而rqt_graph它是一个用来显示系统计算图对工具

![在这里插入图片描述](https://img-blog.csdnimg.cn/20201117001729704.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70#pic_center)


可以看到，这个界面很清晰的列出来了ros系统里面运行的2个节点。


### rosbag(话题记录与话题复现)
![在这里插入图片描述](https://img-blog.csdnimg.cn/20201117011144278.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3NlbmlvckM=,size_16,color_FFFFFF,t_70#pic_center)

`-a`的意思是-all选择记录全部数据

`-O`的意思是将这记录的数据保存为一个名字叫cmd_record的文件

