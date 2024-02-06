## Map简介

Map 映射是一种类似于字典的数据结构。它是(键，值)对的序列，其中只有单个值与每个唯一键相关联。它通常被称为关联数组。

在映射中，键值通常用于对元素进行排序。对于映射数据类型的键和值可以不同，它表示为：

```cpp
typedef pair<const Key, T> value_type;
```

Maps 映射通常实现为二叉搜索树。

零大小的映射也是有效的。在这种情况下，map.begin()和map.end()指向相同的位置。

## Map用法以及具体实现方式

### 构造函数

#### 1、默认构造，构造一个包含零个元素的空映射

C++ 构造函数std::map::map() 构造一个包含零个元素的空映射。

时间复杂度：O(1)

代码如下：

```cpp
#include<iostream>
#include<map>

int main(){
    std::map<char,int> m;

    std::cout<<"Size of map = " <<m.size() <<std::endl;
    
    return 0;
}
```

终端输出如下：

```shell
Size of map = 0
```

#### 2、填充构造，构造一个包含first到last范围内的元素数量的映射

C++构造函数std::map::map()构造一个包含first到last范围内的元素数量的映射。

时间复杂度：O(n)

代码如下：

```cpp
#include<iostream>
#include<map>

int main(){
    std::map<char,int> m1 ={
        {'a',1},
        {'b',2},
        {'c',3},
        {'d',4},
        {'e',5}
    };

    std::map<char,int> m2(m1.begin(),m1.end());

    std::cout<< "Map contains following elements" <<std::endl;

    for (auto it = m2.begin();it != m2.end(); ++it){
        std::cout<< it->first <<" = " << it->second <<std::endl;
    }
    return 0;
}
```

终端输出如下：

```shell
Map contains following elements
a = 1
b = 2
c = 3
d = 4
e = 5
```
