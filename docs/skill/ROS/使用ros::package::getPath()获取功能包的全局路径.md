- C++版本：

    ```cpp
    // 需要包含头文件
    #include <ros/package.h>

    // 获取功能包全局路径
    std::string path = ros::package::getPath("turtle_control");  //turtle_control是我的功能包名
        
    std::cout<<path<<std::endl;
    ```

- python版本：

    安装功能包

    pip install rospkg

    ```python
    import rospkg

    # 创建 rospkg.RosPack 实例
    rospack = rospkg.RosPack()

    # 获取功能包全局路径
    path = rospack.get_path('turtle_control')  # turtle_control 是你的功能包名

    print(path)

    ```
