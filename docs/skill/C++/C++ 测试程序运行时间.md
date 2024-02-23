## 使用C++ clock()函数

C++ 库提供了 clock()函数来获得该时刻的时间

```cpp
int main() {
    
    clock_t start, end;
  
    start = clock();    //获取该时刻的时间，单位/ms
  
    //省略。。。。
  
    end = clock();      //获取该时刻的时间，单位/ms

    cout<<"Run time: "<<(double)(end - start) / CLOCKS_PER_SEC<<"S"<<endl;  //使用 end 后获得的时间减去 start 开始时获得的时间 / CLOCKS_PER_SEC

    return 0;
    }
```

## chrono库计算程序运行时间

```cpp
#include<chrono>
#include<thread>
std::chrono::steady_clock::time_point start;
std::chrono::steady_clock::time_point end ;
start = std::chrono::steady_clock::now();
..........
end = std::chrono::steady_clock::now();
std::chrono::duration<double> duration = end - start;
chargeSingleTime = duration.count();  //计算start与end的时间间隔
..........
std::this_thread::sleep_for(std::chrono::milliseconds(100)); //延时100毫秒
```

## 获取当前时间，需要获取到年月日时分秒毫秒

使用C++标准库中的 `<chrono>` 和 `<ctime>` 头文件来获取当前时间，并以年、月、日、时、分、秒和毫秒的形式表示。下面是一个示例代码:

```cpp
#include <iostream>
#include <chrono>
#include <ctime>

int main() {
    // 获取当前系统时间
    auto now = std::chrono::system_clock::now();
    
    // 将时间转换为time_t类型
    std::time_t time = std::chrono::system_clock::to_time_t(now);

    // 使用tm结构体获取年、月、日、时、分、秒
    std::tm tm_time = *std::localtime(&time);

    // 获取毫秒
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    // 输出年月日时分秒毫秒
    std::cout << "年: " << tm_time.tm_year + 1900 << std::endl;
    std::cout << "月: " << tm_time.tm_mon + 1 << std::endl;
    std::cout << "日: " << tm_time.tm_mday << std::endl;
    std::cout << "时: " << tm_time.tm_hour << std::endl;
    std::cout << "分: " << tm_time.tm_min << std::endl;
    std::cout << "秒: " << tm_time.tm_sec << std::endl;
    std::cout << "毫秒: " << milliseconds << std::endl;

    return 0;
}
```

代码解释：这段代码首先获取当前系统时间，然后将其转换为 time_t 类型以便于处理。接着，使用 tm 结构体从 time_t 中提取年、月、日、时、分和秒的信息。最后，使用 `<chrono>` 头文件中的 duration_cast 函数获取毫秒并输出。

请注意，这个示例中的年份（tm_year）是从1900年开始计算的，月份（tm_mon）是从0开始计算的，因此需要分别加上1900和1来得到实际的年份和月份。此外，tm 结构体中的秒字段（tm_sec）表示的是0到59之间的秒数。
