## 使用源码安装

从ros'wiki上下载[robot_pose_ekf](https://github.com/ros-planning/robot_pose_ekf)功能包，放入工作空间后编译出现如下错误：

```shell
-- Checking for module 'orocos-bfl'
--   No package 'orocos-bfl' found
CMake Error at /usr/share/cmake-3.10/Modules/FindPkgConfig.cmake:419 (message):
  A required package was not found
Call Stack (most recent call first):
  /usr/share/cmake-3.10/Modules/FindPkgConfig.cmake:597 (_pkg_check_modules_internal)
  robot_pose_ekf/CMakeLists.txt:6 (pkg_check_modules)

-- Configuring incomplete, errors occurred!
See also "/home/zdg/ydlidar_ws/build/CMakeFiles/CMakeOutput.log".
See also "/home/zdg/ydlidar_ws/build/CMakeFiles/CMakeError.log".
Invoking "cmake" failed
```

折腾了老半天，知道bfl（贝叶斯滤波库）是第三方库，那么就需要从外部去下载，那些网上的教程说`install ros-melodic-bfl`，`install ros-melodic-navigation`什么的都试过了，没用，到头还是会报无法定位软件包ros-melodic-bfl的错误。

我的解决方法是从源码安装：

<https://aur.archlinux.org/packages/ros-melodic-bfl>

下载压缩包后，解压，进入文件夹 bfl-release-release-melodic-bfl-0.8.0-0

接下来就是经典的安装操作：

```shell
mkdir build
cd build
cmake ..
make 
sudo make install
```

再到工作空间进行编译，编译通过。

## 使用二进制方式安装

```bash
sudo apt install ros-noetic-robot-pose-ekf
```
