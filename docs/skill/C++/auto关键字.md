## C++11 auto 关键字介绍

该关键字在 C++11 中引入， auto 使得开发人员能够声明变量而无需显示指定其数据类型。相反，编译器从初始化表达式中推导出变量的类型。此功能非常方便，因为它简化了代码并使其更具可读性，特别是当数据类型特别复杂时。

## 使用 auto 声明变量

以下是 auto 声明变量的方法：

```cpp
// 将 int 值存储在 auto 变量中
auto var_1 = 5;
// 将字符存储在 auto 变量中
auto var_2 = 'C';

std::cout << var_1 << std::endl;
std::cout << var_2 << std::endl;
```

在上面的示例中， var_1 被推导为 int 类型， var_2 被推导为 char 类型。

## 将 auto 与不同类型一起使用

 auto 关键字用途广泛，可以与任意类型一起使用，包括函数或迭代器。例如，您可以将lanbda函数存储在 auto 变量中，如下所示：

```cpp
// 将 Lambda 函数存储在 auto 变量中
auto fun_sum = [](int a, int b) {
    return a + b;
};

std::cout << fun_sum(4, 5) << std::endl;
```

## 简单复杂类型声明

 auto 它的主要优点之一是当您处理写起来很麻烦的类型时，请看下面这个例子：

```cpp
std::map<std::string, std::string> mapOfStrs;

// 将数据插入到地图中
mapOfStrs.insert({"first", "1"});
mapOfStrs.insert({"sec", "2"});
mapOfStrs.insert({"third", "3"});
```

要迭代此映射并显示其内容，传统上您需要声明一个迭代器，如下所示：

```cpp
// 遍历地图并显示所有数据
for (std::map<std::string, std::string>::iterator it = mapOfStrs.begin(); it != mapOfStrs.end(); ++it)
{
    std::cout << it->first << "::" << it->second << std::endl;
}
```

然而，有了 auto 后，这变得更加简单：

```cpp
// 使用 auto 进行迭代
for (auto itr = mapOfStrs.begin(); itr != mapOfStrs.end(); ++itr)
{
    std::cout << itr->first << "::" << itr->second << std::endl;
}
```

## 使用 auto 需要注意的事项

**1、类型不变形**

初始化 auto 变量后，您可以更改其值，但不能更改其类型：

```cpp
auto x = 1;
x = "dummy"; // 将导致编译时错误，因为 'x' 是 int 类型
```

**2、初始化要求**

 auto 变量不能保持未初始化状态，因为它的类型是从初始化器推导出来的：

```cpp
auto a; // 这会导致编译时错误
```

**3、从函数返回 auto**

当您想从函数返回一个 auto 变量时，可以使用尾置返回类型(C++11引入的新语法，它允许在函数体之后指定返回类型)：

```cpp
// 尽管使用了 auto 关键字来推断返回类型，但是通过 -> int 明确表明函数的返回类型为 int
auto sum(int x, int y) -> int {
    return x + y;
}

// 调用返回“auto”的函数
auto value = sum(3, 5);
```

这里使用了 C++11 引入的新的尾置返回类型语法，即  -> int ，它允许在函数体之后指定返回类型。在这个例子中，尽管使用了 auto 关键字来推断返回类型，但是通过 -> int 明确表明函数的返回类型为 int。

最后，通过调用 sum(3, 5)，将得到的和赋值给变量 value。由于使用了 auto 关键字，编译器会根据函数的返回类型自动推断出 value 的类型为 int 。

## 概括总结

 auto 关键字是C++11中的一项强大功能，可以简化代码并使其更具适应性。它减少了复杂类型声明的冗长性，从而使代码更清晰且更易于维护。虽然在任何地方都使用 auto 很诱人，但重要的是要记住，显示类型声明有时可以使代码更易于理解，特别是当类型不能立刻从上下文中清楚的看出时。明智的使用 auto 在简洁和清晰之间取得适当的平衡。



