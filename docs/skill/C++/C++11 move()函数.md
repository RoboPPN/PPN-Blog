move()函数作用：将左值强制转换为右值并且得到堆内存的所有权。

```cpp
#include <iostream>
#include <memory>
using namespace std;

unique_ptr<int> func()
{
    return unique_ptr<int>(new int(520));
}

int main()
{
    // 通过构造函数初始化
    unique_ptr<int> ptr1(new int(10));
    // 通过转移所有权的方式初始化
    unique_ptr<int> ptr2 = move(ptr1);  //将左值强制转换为右值并且得到堆内存的所有权
    unique_ptr<int> ptr3 = func();

    return 0;
}
```
