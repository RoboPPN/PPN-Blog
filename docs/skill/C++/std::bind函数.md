## 什么是 std::bind?

 std::bind 函数是C++标准库中的一个函数模板，用于创建一个可调用对象（函数对象或函数指针）的绑定副本。它允许你将函数的参数绑定的值或其他可调用对象，从而部分应用函数或改变函数的参数顺序。

## 使用 std::bind 绑定参数

让我们考虑一个两数相加的简单函数：

```cpp
#include<functional>
int add(int first, int second) {
    return first + second;
}
```

使用 std::bing ，您可以创建一个新的函数对象，代码如下：

```cpp
auto add_func = std::bind(&add, _1, _2);
```

这里， add_func 现在表示一个函数对象，可以用两个参数调用，就像 add 一样。 _1 和 _2 是 std::placeholders 命名空间中的占位符，它们被传递给 add_func 的参数所替换。

```cpp
std::cout << add_func(4, 5); // 这将输出 9 
```

## 使用 std::bind 绑定和重新排列参数

假设我们有一种情况，第一个参数 add 应该始终是 12 ，第二个参数应该在调用时指定：

```cpp
auto new_add_func = std::bind(&add, 12, _1);
```

调用 new_add_func(5) 将在内部调用 add(12,5) ：

```cpp
std::cout << new_add_func(5); // 输出 17
```

因此，当我们调用 new_add_func(5) 时，它将在内部调用 add() 函数，第一个参数始终为 12 ，第二个参数为 5 ，并作为实参传递。它将返回 17 。

如果你想重新排列参数怎么办？ std::bind 也允许你这样做：

```cpp
auto mod_add_func = std::bind(&add, _2, _1);
```

 _1 表示第一个传递的参数， _2 表示第二个传递的参数。现在，当通过 std::bind 构造一个新的函数对象时，我们通过在底层函数中先传递 _2 ，后传递 _1 来改变参数的顺序。

现在， mod_add_func(12,15) 将在内部调用 add(15,12) ：

```cpp
std::cout << mod_add_func(12, 15); // 输出 27
```

## 将 std::bind 与 STL 算法结合使用

 std::bind 它的优点之一是能够通过调整函数签名来简化 STL 算法的使用。考虑以下函数，该函数检查一个数字是否可以被另一个数字整除：

```cpp
bool divisible(int num, int den) {
    return num % den == 0;
}
```

该函数接受两个参数并检查 num 是否可以被 den 整除。要使这个函数与 std::count_if 一起工作，需要一个接受单个参数的函数。

### 方法一：手动循环

第一种方法手动迭代数组中的每个元素并应用该 divisible 函数。

```cpp
int approach_1() 
{
    int arr[10] = {1, 20, 13, 4, 5, 6, 10, 28, 19, 15};
    int count = 0;
    for (int i = 0; i < sizeof(arr)/sizeof(int); i++) 
    {
        if (divisible(arr[i], 5))
            count++;
    }
    return count;
}
```

### 方法二：使用 std::count_if 和 std::bind

第二种方法通过使用 std::count_if 和 std::bind 来创建一个函数对象来检查数字是否可以被 5 整除，从而简化了代码：

```cpp
int approach_2() 
{
    int arr[10] = {1, 20, 13, 4, 5, 6, 10, 28, 19, 15};
    auto is_divisible_by_5 = std::bind(&divisible, std::placeholders::_1, 5);

    return std::count_if(arr, arr + sizeof(arr)/sizeof(int), is_divisible_by_5);
}
```

在上面的代码中， std::bind(&divisible, std::placeholders::_1, 5) 创建一个新的函数对象，该对象接受一个参数并检查它是否能被 5 整除。这是可行的，因为`std::placeholders::_1`充当 std::count_if 将传递给谓词的参数的占位符。

调用 std::count_if 时，它将遍历数组并对每个元素应用 is_divisible_by_5 谓词。这种方法更简洁，并且利用了 C++ 标准款的强大功能，可以少写多做。

值得注意的是，在 C++11 及更高版本中，你可以使用 lambda 表达式实现类似的功能，这些表达式通常更具可读性：

```cpp
int approach_3() 
{
    int arr[10] = {1, 20, 13, 4, 5, 6, 10, 28, 19, 15};
    return std::count_if(std::begin(arr), std::end(arr), [](int value) { return divisible(value, 5); });
}
```

## std::bind 返回什么

 std::bind 返回一个可调用的函数对象。该对象可以存储在 auto 变量中或直接使用。或者，它可以存储在一个 std::function 对象中以供后续使用：

```cpp
std::function<int(int)> mod_add_funcObj = std::bind(&add, 20, _1);
```

## 使用 std::bind 完整示例

这是 std::bind 实际演示的完整代码：

```cpp
#include <iostream>
#include <functional>
#include <algorithm>

// For placeholders _1, _2, ...
using namespace std::placeholders;

int add(int first, int second) {
    return first + second;
}

bool divisible(int num, int den) {
    return num % den == 0;
}

int main() {
    // // 演示绑定和重新排列
    auto new_add_func = std::bind(&add, 12, _1);
    std::cout << new_add_func(5) << std::endl; // Outputs 17

    auto mod_add_func = std::bind(&add, _2, _1);
    std::cout << mod_add_func(12, 15) << std::endl; // Outputs 27

    // 使用 std::function 进行绑定
    std::function<int (int)> mod_add_funcObj = std::bind(&add, 20, _1);
    std::cout << mod_add_funcObj(15) <<std::endl; // Outputs 35

    // 计算数组中5的倍数
    int arr[10] = {1, 20, 13, 4, 5, 6, 10, 28, 19, 15};
    auto divisible_by_5 = std::bind(&divisible, _1, 5);
    int count = std::count_if(arr, arr + sizeof(arr)/sizeof(int), divisible_by_5);
    std::cout << count << std::endl; // 输出可被 5 整除的元素数量

    return 0;
}
```

输出：

```cpp
17
27
35
4
```

## 概括总结

 std::bind 是 C++ 中一项强大功能，允许您通过修复某些函数的参数或更改其顺序来从现有函数创建自定义可调用对象。当使用需要特定函数签名的 STL 算法时，这尤其有用。使用 std::bind ，可以使涉及函数的复杂任务变得简单和简洁，从而提高代码的可读性和灵活性。
