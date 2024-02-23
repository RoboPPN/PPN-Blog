## ROS添加自定义消息类型，如何修改CMakeLists.txt的内容

在写CMakeLists.txt时，如要添加自定义消息类型，在find_package加的是message_generation，在catkin_package加的是`message_runtime`，示例如下：

find_package部分：

```txt
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
```

catkin_package部分：

```python
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_lite6
 CATKIN_DEPENDS roscpp std_msgs std_srvs xarm_msgs message_runtime
#  DEPENDS system_lib
)
```

给generate_messages取消注释：

```python
generate_messages(
  DEPENDENCIES
  std_msgs   xarm_msgs  std_srvs
)
```

在package.xml加的是

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

完成上面3步，编译即可通过。
