## 可变参数模板介绍

在 C++11 之前，如果你想要一个可以接受未指定数量的参数函数，则必须求助于函数重载等解决方案，或者使用 C 风格的可变参数函数与 va_list ，使用可变参数模板，就可以拥有更简洁且类型安全的方法。

可变参数模板是 C++ 中引入的一项强大功能，允许程序员编写可以采用任意数量、任意类型的参数的函数和类。这对于代码的灵活性和可重用性具有重大影响。

## 可变参数模板背后的基本思想

让我们考虑一个示例，您需要一个 log() 可以接受可变数量参数并将它们打印到控制台的函数。例如：

```cpp
log(1, 4.3, "Hello");
log('a', "test", 78, 5);

class Student;
Student obj;

log(3, obj);
```

对于log()函数，有两个关键要求：

1、它必须接受任何类型的参数。

2、它必须接受可变数量的参数。

为了满足第一点要求，我们通常会使用模板函数，代码如下：

```cpp
template<typename T>
void log(T obj) 
{
    std::cout << obj;
}
```

然而，这个函数只接受一个参数。为了满足第二点要求，我们需要一个可变参数模板函数。

## 可变参数模板函数的声明

可以接受任意数量参数的函数定义如下：

```cpp
template<typename T, typename... Args>
void log(T first, Args... args);
```

在上面的声明中，Args... 代表可变数量的模板参数。

## 实现可变参数函数

定义可变参数模板函数可能很棘手，因为你无法直接访问可变数量的参数。因此你需要将递归与C++的类型推导机制一起使用：

```cpp
template<typename T, typename... Args>
void log(T first, Args... args) 
{
    std::cout << first << " , ";
    if constexpr (sizeof...(args) > 0) 
    {
        // 递归调用log函数，处理剩余的可变参数
        log(args...);
    }
}
```

**代码解释：**

- `template<typename T, typename... Args>`：这是一个模板声明，它引入了两个模板参数 T 和 Args 。 T 是第一个参数的类型，而 Args 是剩余参数的类型包（即可变参数）。
- `void log(T first, Args... args)`：log函数的声明，它接受一个 first 的参数，它是参数包的第一个元素，以及一个可变参数包 args ，用于处理额外的参数。
- `std::cout << first << " , ";`：将第一个参数 first 打印出来。
- `if constexpr (sizeof...(args) > 0)`:这是一个编译时的条件语句，用于检查可变参数包 args 中是否还有其他参数。 sizeof...(args) 返回参数包中的参数数量。如果数量大于零，那么进入条件块。这种实现方式利用了 C++17 引入的 if constexpr 特性，使得编译器在编译时可以选择性的包含或排除某些代码块，而不是在运行时进行条件检查。
- `log(args...);`：在条件块内，递归调用 log 函数，处理剩余的可变参数。

**它的工作原理如下：**

1、打印第一个参数。

2、使用剩余的参数递归调用。

3、当log()不带参数调用时，递归完成并停止打印。

**调用堆栈示例**

当调用`log(2, 3.4, "aaa");`，内部会发生以下情况：

1、`log(int, double, const char*)`被实例化并打印 2，然后调用 log(double, const char*) 。

2、`log(double, const char*)`被实例化并打印 3.4 ，然后调用 log(const char*) 。

3、`log(const char*)`被实例化并打印 "aaa" ，然后 log() 不带参数调用。

4、递归停止，因为没有 log() 不带参数的重载函数。

## 可变参数模板完整实例

以下是编写和使用log()函数的方法：

```cpp
#include <iostream>

// 结束可变参数模板函数递归的函数
void log() {
    // 这可以为空或用于打印标记输出结束的内容。
}

template<typename T, typename... Args>
void log(T first, Args... args) 
{
    std::cout << first;
    if constexpr (sizeof...(args) > 0)
    {
        std::cout << " , ";
        log(args...);
    }
    else
    {
        std::cout << std::endl; // 最后一个元素的新行
    }
}

int main()
{
    // 使用 3 个参数调用 log() 函数
    log(1 , 4.3 , "Hello");

    // 使用 4 个参数调用 log() 函数
    log('a', "test", 78L, 5);

    // 使用 2 个参数调用 log() 函数
    log("sample", "test");

    return 0;
}
```

**输出：**

```cpp
1 , 4.3 , Hello
a , test , 78 , 5
sample , test
```

## 关于可变参数模板的要点

- 使用可变参数模板时，递归是处理参数的常见模式。
- 注意使用递归调用的次数，太多可能会导致堆栈溢出。
- 从C++17开始，可以使用折叠表达式来处理参数，这样可以更高效、更简洁。

## 概括总结

通过使用可变参数模板，开发人员可以通过一种健壮且类型安全的方式来创建高度灵活且可以处理任意数量、任意类型的函数和类。此功能是对C++语言的强大补充，极大地扩展了泛型编程的潜力。
