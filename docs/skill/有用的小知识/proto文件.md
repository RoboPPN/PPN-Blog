
### proto文件简介
GoogleProtocol Buffers 简称 Protobuf,是 Google 公司内部的混合语言数据标准.它提供一种轻量,高效的结构化数据存储结构。

**为什么要使用protobuf**
- 官方文档中提到一些protobuf的优点, protobuf灵活高效的结构化数据存储格式.方便用于序列化, 适合做RPC的数据交换；

- 相比 XML , protobuf 比 XML 更小、更快、更简单.仅需要写一个 `.proto` 文件描述需要的数据结构, protobuf会帮助你实现相关类和方法(自动化多好!)；

- 目前提供 C++, Java, Python, Go, C#等多种语言 的 API ；


### 基本定义

下面的例子是在官方的RTS包常见的proto文件。

```cpp title="RTS/global_planner/proto/global_planner_config.proto"
syntax = "proto2";    //指定使用proto2语法
package roborts_global_planner;

message GlobalPlannerConfig {
    repeated string name = 1;
    optional string selected_algorithm = 2;
    required int32 frequency = 3;
    required int32 max_retries = 4;
    required double goal_distance_tolerance = 5;
    required double goal_angle_tolerance = 6;
}
```

- `syntax = "proto2";  `指定使用proto2语法；
- `package roborts_global_planner;`表示当前proto文件属于`roborts_global_planner`包；
- `message`是用于定义消息的关键字，`GlobalPlannerConfig`是消息名，消息名是根据需要自定义的；
- `repeated`类型是**可重复**；
- `require`类型是**必须的**；
- `optional`类型**可选的**；
- 右边的数字只是规定一下字段的顺序，只要一个消息里的字段序号不要重复就好；


:::tip
如需在其他文件中添加proto文件
在文件名后缀加上`.pb.h`即可。

如下所示：
```cpp
#include"global_planner_config.pb.h"
```
:::