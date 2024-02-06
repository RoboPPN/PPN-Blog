import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '3ad'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'c28'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'c2e'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'd6e'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '9eb'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '05c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '721'),
    exact: true
  },
  {
    path: '/about',
    component: ComponentCreator('/about', '473'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', '7df'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', 'b0e'),
    exact: true
  },
  {
    path: '/blog/blog_write',
    component: ComponentCreator('/blog/blog_write', 'd97'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', 'b66'),
    exact: true
  },
  {
    path: '/blog/tags/随笔',
    component: ComponentCreator('/blog/tags/随笔', '9bd'),
    exact: true
  },
  {
    path: '/blog/tags/blog',
    component: ComponentCreator('/blog/tags/blog', '64a'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus',
    component: ComponentCreator('/blog/tags/docusaurus', '57f'),
    exact: true
  },
  {
    path: '/friends/',
    component: ComponentCreator('/friends/', '1d5'),
    exact: true
  },
  {
    path: '/project/',
    component: ComponentCreator('/project/', 'a26'),
    exact: true
  },
  {
    path: '/resource/',
    component: ComponentCreator('/resource/', 'c6e'),
    exact: true
  },
  {
    path: '/search',
    component: ComponentCreator('/search', '2cf'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '662'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '752'),
        routes: [
          {
            path: '/docs/tags',
            component: ComponentCreator('/docs/tags', '0cc'),
            exact: true
          },
          {
            path: '/docs/tags/database',
            component: ComponentCreator('/docs/tags/database', '7e4'),
            exact: true
          },
          {
            path: '/docs/tags/mysql',
            component: ComponentCreator('/docs/tags/mysql', '353'),
            exact: true
          },
          {
            path: '/docs/tags/ros',
            component: ComponentCreator('/docs/tags/ros', '1db'),
            exact: true
          },
          {
            path: '/docs/tags/tf',
            component: ComponentCreator('/docs/tags/tf', 'c7b'),
            exact: true
          },
          {
            path: '/docs/tags/vs-code',
            component: ComponentCreator('/docs/tags/vs-code', '4c2'),
            exact: true
          },
          {
            path: '/docs',
            component: ComponentCreator('/docs', '321'),
            routes: [
              {
                path: '/docs/category/传感器',
                component: ComponentCreator('/docs/category/传感器', '0ea'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/地图',
                component: ComponentCreator('/docs/category/地图', '884'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/定位导航',
                component: ComponentCreator('/docs/category/定位导航', '63a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/多线程',
                component: ComponentCreator('/docs/category/多线程', '32e'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/规划',
                component: ComponentCreator('/docs/category/规划', '96a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/机器学习',
                component: ComponentCreator('/docs/category/机器学习', '7d9'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/激光雷达',
                component: ComponentCreator('/docs/category/激光雷达', '47a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/软件安装环境配置',
                component: ComponentCreator('/docs/category/软件安装环境配置', '0d4'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/有用的小知识',
                component: ComponentCreator('/docs/category/有用的小知识', '8fc'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/autoware',
                component: ComponentCreator('/docs/category/autoware', '097'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/c',
                component: ComponentCreator('/docs/category/c', '3be'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/chatgpt',
                component: ComponentCreator('/docs/category/chatgpt', '6b8'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/git',
                component: ComponentCreator('/docs/category/git', '1ed'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/gps',
                component: ComponentCreator('/docs/category/gps', 'b55'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/imu',
                component: ComponentCreator('/docs/category/imu', 'f28'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/math',
                component: ComponentCreator('/docs/category/math', '2de'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/moveit机械臂',
                component: ComponentCreator('/docs/category/moveit机械臂', 'a77'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/mysql',
                component: ComponentCreator('/docs/category/mysql', 'c31'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/opencv',
                component: ComponentCreator('/docs/category/opencv', 'f3d'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/python',
                component: ComponentCreator('/docs/category/python', '3b1'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/ros',
                component: ComponentCreator('/docs/category/ros', '3a6'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/ubuntu',
                component: ComponentCreator('/docs/category/ubuntu', '769'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/category/vscode问题合集',
                component: ComponentCreator('/docs/category/vscode问题合集', '683'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/docusaurus-comment',
                component: ComponentCreator('/docs/docusaurus-comment', 'd7f'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/docusaurus-component',
                component: ComponentCreator('/docs/docusaurus-component', 'e2c'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/docusaurus-config',
                component: ComponentCreator('/docs/docusaurus-config', '175'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/docusaurus-deploy',
                component: ComponentCreator('/docs/docusaurus-deploy', 'd08'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/docusaurus-guides',
                component: ComponentCreator('/docs/docusaurus-guides', 'b07'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/docusaurus-plugin',
                component: ComponentCreator('/docs/docusaurus-plugin', 'ecf'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/docusaurus-search',
                component: ComponentCreator('/docs/docusaurus-search', '9af'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/docusaurus-style',
                component: ComponentCreator('/docs/docusaurus-style', 'df7'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/note-introduction',
                component: ComponentCreator('/docs/note-introduction', '1dd'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/ros-learning-tf',
                component: ComponentCreator('/docs/ros-learning-tf', '5ab'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/地图/建立静态地图',
                component: ComponentCreator('/docs/skill/地图/建立静态地图', 'dd6'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/地图/costmap',
                component: ComponentCreator('/docs/skill/地图/costmap', '3fb'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/地图/navigation切换地图的两种方法',
                component: ComponentCreator('/docs/skill/地图/navigation切换地图的两种方法', 'f96'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/定位导航/导航模块介绍',
                component: ComponentCreator('/docs/skill/定位导航/导航模块介绍', '5b5'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/定位导航/定位简介',
                component: ComponentCreator('/docs/skill/定位导航/定位简介', 'abf'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/定位导航/基于反光板的全局定位',
                component: ComponentCreator('/docs/skill/定位导航/基于反光板的全局定位', 'e2b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/定位导航/自适应蒙特卡罗定位',
                component: ComponentCreator('/docs/skill/定位导航/自适应蒙特卡罗定位', '98f'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/定位导航/自主式移动机器人改进方案',
                component: ComponentCreator('/docs/skill/定位导航/自主式移动机器人改进方案', 'a81'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/定位导航/cartographer学习',
                component: ComponentCreator('/docs/skill/定位导航/cartographer学习', '8c1'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/多线程/处理日期和时间的 chrono 库',
                component: ComponentCreator('/docs/skill/多线程/处理日期和时间的 chrono 库', '6c8'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/多线程/命名空间this_thread',
                component: ComponentCreator('/docs/skill/多线程/命名空间this_thread', 'a47'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/多线程/如何在C++11中让线程休眠',
                component: ComponentCreator('/docs/skill/多线程/如何在C++11中让线程休眠', '997'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/多线程/线程同步',
                component: ComponentCreator('/docs/skill/多线程/线程同步', 'ff4'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/多线程/C++11使用条件变量进行线程间的事件处理',
                component: ComponentCreator('/docs/skill/多线程/C++11使用条件变量进行线程间的事件处理', 'bdf'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/多线程/C++11线程类',
                component: ComponentCreator('/docs/skill/多线程/C++11线程类', 'abf'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/各大全局路径规划算法简介',
                component: ComponentCreator('/docs/skill/规划/各大全局路径规划算法简介', '66d'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/局部路径规划',
                component: ComponentCreator('/docs/skill/规划/局部路径规划', '337'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/路径规划简介',
                component: ComponentCreator('/docs/skill/规划/路径规划简介', 'e17'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/A*算法',
                component: ComponentCreator('/docs/skill/规划/A*算法', '6e3'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/Dijkstra',
                component: ComponentCreator('/docs/skill/规划/Dijkstra', '70f'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/DWA',
                component: ComponentCreator('/docs/skill/规划/DWA', 'fa3'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/move_base笔记',
                component: ComponentCreator('/docs/skill/规划/move_base笔记', '0a5'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/TEB',
                component: ComponentCreator('/docs/skill/规划/TEB', '561'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/Teb参数优化',
                component: ComponentCreator('/docs/skill/规划/Teb参数优化', '128'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/规划/Teb算法',
                component: ComponentCreator('/docs/skill/规划/Teb算法', 'd43'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/机器学习/机器学习概述',
                component: ComponentCreator('/docs/skill/机器学习/机器学习概述', '6aa'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/软件安装&环境配置/编译安装robot_pose_ekf功能包',
                component: ComponentCreator('/docs/skill/软件安装&环境配置/编译安装robot_pose_ekf功能包', 'c2b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/软件安装&环境配置/妙算MANIFOLD 2-G系统还原、使用教程',
                component: ComponentCreator('/docs/skill/软件安装&环境配置/妙算MANIFOLD 2-G系统还原、使用教程', 'dd6'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/软件安装&环境配置/在vscode上搭建ROS环境',
                component: ComponentCreator('/docs/skill/软件安装&环境配置/在vscode上搭建ROS环境', '230'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/软件安装&环境配置/nodejs16以上版本安装以及yarn、npm的安装',
                component: ComponentCreator('/docs/skill/软件安装&环境配置/nodejs16以上版本安装以及yarn、npm的安装', '9d4'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/软件安装&环境配置/RoboWare的安装',
                component: ComponentCreator('/docs/skill/软件安装&环境配置/RoboWare的安装', '05d'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/软件安装&环境配置/Ubuntu配置opencv4',
                component: ComponentCreator('/docs/skill/软件安装&环境配置/Ubuntu配置opencv4', '4e2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/有用的小知识/多传感器信息融合理论',
                component: ComponentCreator('/docs/skill/有用的小知识/多传感器信息融合理论', 'b45'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/有用的小知识/搜索小技巧',
                component: ComponentCreator('/docs/skill/有用的小知识/搜索小技巧', 'a16'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/有用的小知识/NTFS-FAT32-exFAT文件系统',
                component: ComponentCreator('/docs/skill/有用的小知识/NTFS-FAT32-exFAT文件系统', '1c0'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/有用的小知识/proto文件',
                component: ComponentCreator('/docs/skill/有用的小知识/proto文件', 'e30'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/autoware/Autoware学习笔记-1-Autoware介绍及安装',
                component: ComponentCreator('/docs/skill/autoware/Autoware学习笔记-1-Autoware介绍及安装', '129'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/autoware/ndt_matching',
                component: ComponentCreator('/docs/skill/autoware/ndt_matching', 'fee'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/抽象类接口',
                component: ComponentCreator('/docs/skill/C++/抽象类接口', '906'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/错误流与日志流',
                component: ComponentCreator('/docs/skill/C++/错误流与日志流', '4f3'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/当回调函数写在类中，如何写subscribe的回调函数',
                component: ComponentCreator('/docs/skill/C++/当回调函数写在类中，如何写subscribe的回调函数', '814'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/多态&虚函数',
                component: ComponentCreator('/docs/skill/C++/多态&虚函数', '039'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/反斜杠符',
                component: ComponentCreator('/docs/skill/C++/反斜杠符', '93d'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/宏定义',
                component: ComponentCreator('/docs/skill/C++/宏定义', '8d4'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/继承',
                component: ComponentCreator('/docs/skill/C++/继承', '757'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/科学计数法',
                component: ComponentCreator('/docs/skill/C++/科学计数法', 'fcc'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/内联函数',
                component: ComponentCreator('/docs/skill/C++/内联函数', 'dae'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/判断表达式的使用',
                component: ComponentCreator('/docs/skill/C++/判断表达式的使用', '51b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/使用new在堆创建对象',
                component: ComponentCreator('/docs/skill/C++/使用new在堆创建对象', '687'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/数据抽象数据封装',
                component: ComponentCreator('/docs/skill/C++/数据抽象数据封装', 'a29'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/头文件定义的通用模板',
                component: ComponentCreator('/docs/skill/C++/头文件定义的通用模板', 'e80'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/友元函数',
                component: ComponentCreator('/docs/skill/C++/友元函数', '81f'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/在指定路径下自动创建文件夹',
                component: ComponentCreator('/docs/skill/C++/在指定路径下自动创建文件夹', '88b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/指向类、结构体的指针',
                component: ComponentCreator('/docs/skill/C++/指向类、结构体的指针', 'cb4'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/重载函数',
                component: ComponentCreator('/docs/skill/C++/重载函数', '1a7'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/auto关键字',
                component: ComponentCreator('/docs/skill/C++/auto关键字', 'fb2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/C++ 测试程序运行时间',
                component: ComponentCreator('/docs/skill/C++/C++ 测试程序运行时间', 'f8e'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/C++11 move()函数',
                component: ComponentCreator('/docs/skill/C++/C++11 move()函数', 'bad'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/C++可变参数模板',
                component: ComponentCreator('/docs/skill/C++/C++可变参数模板', 'eef'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/C++使用ifdef控制部分代码开或关',
                component: ComponentCreator('/docs/skill/C++/C++使用ifdef控制部分代码开或关', '19e'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/C++智能指针',
                component: ComponentCreator('/docs/skill/C++/C++智能指针', '134'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/CMake学习笔记',
                component: ComponentCreator('/docs/skill/C++/CMake学习笔记', 'b65'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/const与define',
                component: ComponentCreator('/docs/skill/C++/const与define', 'c48'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/gcc编译器和g++编译器有何区别',
                component: ComponentCreator('/docs/skill/C++/gcc编译器和g++编译器有何区别', '383'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/Google代码风格C++篇',
                component: ComponentCreator('/docs/skill/C++/Google代码风格C++篇', '71a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/map',
                component: ComponentCreator('/docs/skill/C++/map', 'd3b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/std::bind函数',
                component: ComponentCreator('/docs/skill/C++/std::bind函数', '7d1'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/std::ofstream  与 std::fstream',
                component: ComponentCreator('/docs/skill/C++/std::ofstream  与 std::fstream', '57b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/C++/vector',
                component: ComponentCreator('/docs/skill/C++/vector', '3c4'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ChatGPT/ChatGPT注册账号教程',
                component: ComponentCreator('/docs/skill/ChatGPT/ChatGPT注册账号教程', 'a8e'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/database/mysql',
                component: ComponentCreator('/docs/skill/database/mysql', '28c'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/git/超时-无法读取远程仓库',
                component: ComponentCreator('/docs/skill/git/超时-无法读取远程仓库', '6ef'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/git/git分支管理',
                component: ComponentCreator('/docs/skill/git/git分支管理', '1e7'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/git/git教程',
                component: ComponentCreator('/docs/skill/git/git教程', 'de2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/HTML&CSS/CSS盒子模型',
                component: ComponentCreator('/docs/skill/HTML&CSS/CSS盒子模型', '7f9'),
                exact: true
              },
              {
                path: '/docs/skill/HTML&CSS/CSS学习笔记',
                component: ComponentCreator('/docs/skill/HTML&CSS/CSS学习笔记', 'c1f'),
                exact: true
              },
              {
                path: '/docs/skill/HTML&CSS/HTML常用元素',
                component: ComponentCreator('/docs/skill/HTML&CSS/HTML常用元素', 'd0b'),
                exact: true
              },
              {
                path: '/docs/skill/HTML&CSS/HTML学习笔记',
                component: ComponentCreator('/docs/skill/HTML&CSS/HTML学习笔记', '10c'),
                exact: true
              },
              {
                path: '/docs/skill/Math/最小二乘法',
                component: ComponentCreator('/docs/skill/Math/最小二乘法', '9a2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/MoveIt机械臂/MoveIt入门学习笔记',
                component: ComponentCreator('/docs/skill/MoveIt机械臂/MoveIt入门学习笔记', '1a7'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/MySQL/mysql踩坑记',
                component: ComponentCreator('/docs/skill/MySQL/mysql踩坑记', '849'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/MySQL/MySQL操作指令',
                component: ComponentCreator('/docs/skill/MySQL/MySQL操作指令', 'ba0'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/MySQL/mysql数据库迁移',
                component: ComponentCreator('/docs/skill/MySQL/mysql数据库迁移', '15d'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/MySQL/mysql与influxDB的区别',
                component: ComponentCreator('/docs/skill/MySQL/mysql与influxDB的区别', '25b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/MySQL/SQL笔记',
                component: ComponentCreator('/docs/skill/MySQL/SQL笔记', 'cea'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/MySQL/Ubuntu20.04安装MySQL和MySQL-workbench并连接云端数据库',
                component: ComponentCreator('/docs/skill/MySQL/Ubuntu20.04安装MySQL和MySQL-workbench并连接云端数据库', 'acd'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/MySQL/workbench学习笔记',
                component: ComponentCreator('/docs/skill/MySQL/workbench学习笔记', 'fca'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/OpenCV/OpenCV检测并解析二维码',
                component: ComponentCreator('/docs/skill/OpenCV/OpenCV检测并解析二维码', '395'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Python/互斥锁',
                component: ComponentCreator('/docs/skill/Python/互斥锁', '883'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Python/如何安装turtle库',
                component: ComponentCreator('/docs/skill/Python/如何安装turtle库', '384'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Python/如何在ubuntu上运行python文件',
                component: ComponentCreator('/docs/skill/Python/如何在ubuntu上运行python文件', 'be6'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Python/数据类型检测',
                component: ComponentCreator('/docs/skill/Python/数据类型检测', 'd93'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Python/遇到非ASCII字符问题',
                component: ComponentCreator('/docs/skill/Python/遇到非ASCII字符问题', '611'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Python/async关键字',
                component: ComponentCreator('/docs/skill/Python/async关键字', 'f12'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Python/python对json&yaml的操作',
                component: ComponentCreator('/docs/skill/Python/python对json&yaml的操作', 'bb7'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Python/python简介',
                component: ComponentCreator('/docs/skill/Python/python简介', '628'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/如何在ros-noetic下打开realsense D435相机',
                component: ComponentCreator('/docs/skill/ROS/如何在ros-noetic下打开realsense D435相机', 'ce8'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/使用脚本一键安装ROS',
                component: ComponentCreator('/docs/skill/ROS/使用脚本一键安装ROS', 'f13'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/使用ros::package::getPath()获取功能包的全局路径',
                component: ComponentCreator('/docs/skill/ROS/使用ros::package::getPath()获取功能包的全局路径', '9be'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/四元数',
                component: ComponentCreator('/docs/skill/ROS/四元数', '355'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/添加自定义消息类型，如何修改CMakeLists.txt的内容',
                component: ComponentCreator('/docs/skill/ROS/添加自定义消息类型，如何修改CMakeLists.txt的内容', 'a6b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/专治ROS各种疑难杂症',
                component: ComponentCreator('/docs/skill/ROS/专治ROS各种疑难杂症', 'ef2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/ROS-2浅述',
                component: ComponentCreator('/docs/skill/ROS/ROS-2浅述', 'd7b'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/ROS-melodic的安装与卸载',
                component: ComponentCreator('/docs/skill/ROS/ROS-melodic的安装与卸载', '100'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/ROS常用工具',
                component: ComponentCreator('/docs/skill/ROS/ROS常用工具', '4b6'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/ROS常用指令',
                component: ComponentCreator('/docs/skill/ROS/ROS常用指令', 'a84'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/ROS分布式通讯',
                component: ComponentCreator('/docs/skill/ROS/ROS分布式通讯', '317'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/ROS入门之launch',
                component: ComponentCreator('/docs/skill/ROS/ROS入门之launch', '988'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/spin()与spinOnce()的区别',
                component: ComponentCreator('/docs/skill/ROS/spin()与spinOnce()的区别', 'ed5'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/ROS/TF',
                component: ComponentCreator('/docs/skill/ROS/TF', 'c15'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Camera/迈德威视MV-SUA133GC-T工业相机驱安装',
                component: ComponentCreator('/docs/skill/Sensor/Camera/迈德威视MV-SUA133GC-T工业相机驱安装', '170'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Camera/使用ROS对相机进行内参标定',
                component: ComponentCreator('/docs/skill/Sensor/Camera/使用ROS对相机进行内参标定', '464'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Camera/Azure Kinect DK 深度相机工作原理',
                component: ComponentCreator('/docs/skill/Sensor/Camera/Azure Kinect DK 深度相机工作原理', '70c'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Camera/RealSense相机',
                component: ComponentCreator('/docs/skill/Sensor/Camera/RealSense相机', 'a8a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/GPS/GPS工作原理',
                component: ComponentCreator('/docs/skill/Sensor/GPS/GPS工作原理', 'f1f'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/IMU/IMU工作原理',
                component: ComponentCreator('/docs/skill/Sensor/IMU/IMU工作原理', '72a'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Lidar/初玩Livox Mid-70',
                component: ComponentCreator('/docs/skill/Sensor/Lidar/初玩Livox Mid-70', 'b41'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Lidar/单线激光雷达特征提取实验',
                component: ComponentCreator('/docs/skill/Sensor/Lidar/单线激光雷达特征提取实验', '0e3'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Lidar/激光雷达滤波器',
                component: ComponentCreator('/docs/skill/Sensor/Lidar/激光雷达滤波器', 'f21'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Lidar/激光雷达特征提取与拟合路径',
                component: ComponentCreator('/docs/skill/Sensor/Lidar/激光雷达特征提取与拟合路径', '958'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Lidar/两种激光雷达数据赋值方法',
                component: ComponentCreator('/docs/skill/Sensor/Lidar/两种激光雷达数据赋值方法', '8df'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Lidar/深度相机做伪2D激光雷达',
                component: ComponentCreator('/docs/skill/Sensor/Lidar/深度相机做伪2D激光雷达', 'b42'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Sensor/Lidar/双2d激光雷达数据拼接',
                component: ComponentCreator('/docs/skill/Sensor/Lidar/双2d激光雷达数据拼接', 'f69'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Ubuntu/常用shell脚本指令',
                component: ComponentCreator('/docs/skill/Ubuntu/常用shell脚本指令', 'd63'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Ubuntu/外设端口映射配置',
                component: ComponentCreator('/docs/skill/Ubuntu/外设端口映射配置', '01e'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Ubuntu/一些Ubuntu上好用的工具',
                component: ComponentCreator('/docs/skill/Ubuntu/一些Ubuntu上好用的工具', '0b2'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Ubuntu/专治Ubuntu各种疑难杂症',
                component: ComponentCreator('/docs/skill/Ubuntu/专治Ubuntu各种疑难杂症', 'f15'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Ubuntu/ubuntu常用指令合集',
                component: ComponentCreator('/docs/skill/Ubuntu/ubuntu常用指令合集', '84d'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Ubuntu/Ubuntu自启动程序',
                component: ComponentCreator('/docs/skill/Ubuntu/Ubuntu自启动程序', '5df'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/Ubuntu/vim用法',
                component: ComponentCreator('/docs/skill/Ubuntu/vim用法', 'd12'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/skill/VSCode/ssh远程连接服务器无法跳转函数定义问题',
                component: ComponentCreator('/docs/skill/VSCode/ssh远程连接服务器无法跳转函数定义问题', 'e7f'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/vscode-path-include',
                component: ComponentCreator('/docs/vscode-path-include', '779'),
                exact: true,
                sidebar: "skill"
              },
              {
                path: '/docs/vscode-wrong-define',
                component: ComponentCreator('/docs/vscode-wrong-define', '63f'),
                exact: true,
                sidebar: "skill"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '99f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
