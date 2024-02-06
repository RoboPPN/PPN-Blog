## 什么是CMake？

你或许听过好几种 Make 工具，例如 GNU Make ，QT 的 qmake ，微软的 MS nmake，BSD Make（pmake），Makepp，等等。这些 Make 工具遵循着不同的规范和标准，所执行的 Makefile 格式也千差万别。这样就带来了一个严峻的问题：如果软件想跨平台，必须要保证能够在不同平台编译。而如果使用上面的 Make 工具，就得为每一种标准写一次 Makefile ，这将是一件让人抓狂的工作。

CMake 就是针对上面问题所设计的工具：它首先允许开发者编写一种平台无关的 CMakeList.txt 文件来定制整个编译流程，然后再根据目标用户的平台进一步生成所需的本地化 Makefile 和工程文件，如 Unix 的 Makefile 或 Windows 的 Visual Studio 工程。从而做到“Write once, run everywhere”。显然，CMake 是一个比上述几种 make 更高级的编译配置工具。

### 使用 CMake 生成 Makefile 并编译的流程

在 linux 平台下使用 CMake 生成 Makefile 并编译的流程如下：

1. 编写 CMake 配置文件 CMakeLists.txt 。
2. 执行命令 cmake PATH 或者 ccmake PATH 生成 Makefile（ccmake 和 cmake 的区别在于前者提供了一个交互式的界面）。其中， PATH 是 CMakeLists.txt 所在的目录。
3. 使用 make 命令进行编译。

## CMake 常见语法罗列

CMakeLists.txt 的语法比较简单，由命令、注释和空格组成，其中命令是不区分大小写的。符号 # 后面的内容被认为是注释。命令由命令名称、小括号和参数组成，参数之间使用空格进行间隔。

- PROJECT(hello_cmake)：该命令表示项目的名称是 hello_cmake。
- CMake构建包含一个项目名称，上面的命令会自动生成一些变量，在使用多个项目时引用某些变量会更加容易。比如生成了： PROJECT_NAME 这个变量。
- PROJECT_NAME是变量名，${PROJECT_NAME}是变量值，值为hello_cmake。
- CMAKE_MINIMUM_REQUIRED(VERSION 2.6) ：限定了 CMake 的版本。
- AUX_SOURCE_DIRECTORY(< dir > < variable >)： AUX_SOURCE_DIRECTORY ( . DIR_SRCS)：将当前目录中的源文件名称赋值给变量 DIR_SRCS
- ADD_SUBDIRECTORY(src)： 指明本项目包含一个子目录 src
- SET(SOURCES src/Hello.cpp src/main.cpp)：创建一个变量，名字叫SOURCE。它包含了这些cpp文件。
- ADD_EXECUTABLE(main ${SOURCES })：指示变量 SOURCES 中的源文件需要编译 成一个名称为 main 的可执行文件。 ADD_EXECUTABLE() 函数的第一个参数是可执行文件名，第二个参数是要编译的源文件列表。因为这里定义了SOURCE变量，所以就不需要罗列cpp文件了。等价于命令：ADD_EXECUTABLE(main src/Hello.cpp src/main.cpp)
- ADD_LIBRARY(hello_library STATIC src/Hello.cpp)：用于从某些源文件创建一个库，默认生成在构建文件夹。在add_library调用中包含了源文件，用于创建名称为libhello_library的静态库。
- TARGET_LINK_LIBRARIES( main Test )：指明可执行文件 main 需要连接一个名为Test的链接库。添加链接库。
- TARGET_INCLUDE_DIRECTORIES(hello_library PUBLIC ${PROJECT_SOURCE_DIR}/include)：添加了一个目录，这个目录是库所包含的头文件的目录，并设置库属性为PUBLIC。
- MESSAGE(STATUS “Using bundled Findlibdb.cmake…”)：命令 MESSAGE 会将参数的内容输出到终端。
FIND_PATH () ：指明头文件查找的路径，原型如下：find_path(< VAR > name1 [path1 path2 ...]) 该命令在参数 path* 指示的目录中查找文件 name1 并将查找到的路径保存在变量 VAR 中。
- FIND_LIBRARY()： 同 FIND_PATH 类似,用于查找链接库并将结果保存在变量中。

## 案例（一）：单个源文件

对于简单的项目，只需要写几行代码就可以了。例如，假设现在我们的项目中只有一个源文件 main.cpp ，该程序的用途是计算一个数的指数幂。

### 源文件编写

```cpp
#include <iostream>

int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
```

### CMakeList.txt

```cpp
# 指定运行此配置文件所需的 CMake 的最低版本
cmake_minimum_required(VERSION 2.8)

# 参数值是 Demo1，该命令表示项目的名称是 Demo1 。
project(Demo1)

# 设置
set(CMAKE_CXX_STANDARD 11)

# 将/src目录下名为 main.cpp 的源文件编译成一个名称为 Demo 的可执行文件。
add_executable(Demo src/main.cpp)
```

### 编译项目

```shell
mkdir build
cd build/
cmake ..
```

为了不让编译产生的中间文件污染我们的工程，我们可以创建一个 build 目录进入执行 cmake 构建工具. 如果没有错误， 执行成功后会在 build 目录下产生 Makefile 文件。得到 Makefile 后再使用 `make` 命令编译得到 Demo 可执行文件。

以上就是大致的 cmake 构建运行过程。

## 案例（二）：多个源文件

情况：同一目录，多个源文件

上面的例子只有单个源文件。现在假如把一个自定义函数单独写进一个名为 other.cpp 的源文件里，使得这个工程变成如下的形式：

```shell
├── main.cpp
├── other.cpp
└── other.h

0 directory, 3 files
```

这时候，CMakeLists.txt 可以改成如下的形式：

```cpp
# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (Demo2)

# 指定生成目标
add_executable(Demo main.cpp other.cpp)
```

唯一的改动只是在 add_executable 命令中增加了一个 other.cpp 源文件。这样写当然没什么问题，但是如果源文件很多，把所有源文件的名字都加进去将是一件烦人的工作。更省事的方法是使用 aux_source_directory 命令，该命令会查找指定目录下的所有源文件，然后将结果存进指定变量名。其语法如下：

```cpp
aux_source_directory(<dir> <variable>)
```

因此，可以修改 CMakeLists.txt 如下：

```cpp
# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (Demo2)

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

# 指定生成目标
add_executable(Demo ${DIR_SRCS})
```

这样，CMake 会将当前目录所有源文件的文件名赋值给变量 DIR_SRCS ，再指示变量 DIR_SRCS 中的源文件需要编译成一个名称为 Demo 的可执行文件。

## 案例（三）：多个目录，多个源文件

现在进一步将 other.h 和 other.cpp 文件移动到 other 目录下。

```shell
├── main.cpp
└── other
    ├── other.cpp
    └── other.h

1 directory, 3 files
```

对于这种情况，需要分别在项目根目录和 other 目录里各编写一个 CMakeLists.txt 文件。为了方便，我们可以先将 other 目录里的文件编译成静态库再由 main 函数调用。

根目录中的 CMakeLists.txt ：

```cpp
# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (Demo3)

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

# 添加 math 子目录
add_subdirectory(other)

# 指定生成目标 
add_executable(Demo main.cpp)

# 添加链接库
target_link_libraries(Demo other)
```

该文件添加了下面的内容:

- 使用命令 add_subdirectory 指明本项目包含一个子目录 other，这样 other 目录下的 CMakeLists.txt 文件和源代码也会被处理 。
- 使用命令 target_link_libraries 指明可执行文件 Demo 需要连接一个名为 other 的链接库 。

子目录中的 CMakeLists.txt：

```cpp
# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
aux_source_directory(. DIR_LIB_SRCS)

# 生成链接库
add_library (other ${DIR_LIB_SRCS})
```

在该文件中使用命令 add_library 将 other 目录中的源文件编译为静态链接库。

## 使用 Debug 还是 Release 来调试

让 CMake 支持 gdb 的设置也很容易，只需要指定 Debug 模式下开启 -g 选项：

```cpp
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
```

之后可以直接对生成的程序使用 gdb 来调试。

## ROS CMakeLists.txt代码示例

```shell
#指定构建ROS软件包所需的最低CMake
cmake_minimum_required(VERSION 2.8.3)
#定义ROS功能包的名称
project(charge_ppn)

## 编译为 C++11，支持 ROS Kinetic 和更新版本
add_compile_options(-std=c++11)

# 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  scout_msgs
  message_generation
  yaml-cpp
)


add_message_files(
  FILES
  ChargeData.msg
)

## 使用此处列出的任何依赖项生成添加的消息和服务
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES charge_ppn
 CATKIN_DEPENDS  roscpp rospy std_msgs  scout_msgs message_runtime
#  DEPENDS system_lib
)

###########
## 编译 ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(charge_info_gat_and_set src/charge_info_get_and_set.cpp)

target_link_libraries(charge_info_gat_and_set
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
```

## 学习链接

- [CMake 入门实战](https://www.hahack.com/codes/cmake#%E5%B0%86%E5%85%B6%E4%BB%96%E5%B9%B3%E5%8F%B0%E7%9A%84%E9%A1%B9%E7%9B%AE%E8%BF%81%E7%A7%BB%E5%88%B0-CMake)
- [CMake 教程 | CMake 从入门到应用](https://aiden-dong.gitee.io/2019/07/20/CMake%E6%95%99%E7%A8%8B%E4%B9%8BCMake%E4%BB%8E%E5%85%A5%E9%97%A8%E5%88%B0%E5%BA%94%E7%94%A8/)
