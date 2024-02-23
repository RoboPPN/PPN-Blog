## vector简介

向量是可以改变大小的序列容器。 容器是保存相同类型数据的对象。 序列容器严格按线性顺序存储元素。

Vector 将元素存储在连续的内存位置，并允许使用下标运算符 [] 直接访问任何元素。 与数组不同，vector 可以在运行时根据需要收缩或扩展。 向量的存储是自动处理的。

为了在运行时支持收缩和扩展功能，向量容器可能会分配一些额外的存储空间以适应可能的增长，因此容器的实际容量大于大小。 因此，与数组相比，vector 消耗更多的内存来换取管理存储和以高效方式动态增长的能力。

零大小的向量也是有效的。 在这种情况下，vector.begin() 和 vector.end() 指向相同的位置。 但是调用 front() 或 back() 的行为是未定义的。

## vector函数用法以及具体实现方式

### 构造函数

#### 1、默认构造，构造一个包含零个元素的空容器

C++ 默认构造函数std::vector::vector()构造一个空容器，元素为零。

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

int main(void) {

    // 创建一个名为vec的vector
    std::vector<int> vec; //默认构造

    // vec.size() 的作用是：返回vec中的元素个数
    std::cout << "size of vec = " << vec.size() << std::endl;

    return 0;
}
```

终端输出如下：

```python
size of vec = 0
```

#### 2、填充构造，构造一个包含n元素的容器，并将value分配给每个元素

C++ 填充构造函数std::vector::vector()构造一个大小为 n 的容器并将值 value (如果有提供的话) 分配给容器的每个元素。

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

int main(void) {

    std::vector<int> vec(5,100); //填充构造

    for(int i = 0; i < vec.size(); ++i){
        std::cout<<vec[i]<<std::endl;
    }

    return 0;
}
```

终端输出如下：

```bash
100
100
100
100
100
```

如将上面代码`std::vector<int> vec(5,100);`更改为`std::vector<int> vec(5);`,则终端输出如下：

```bash
0
0
0
0
0
```

可看出如没有填充value值的话，默认用0填充容器。

#### 3、范围构造，构造一个容器，其中包含 first 到 last 范围内的尽可能多的元素

C++ 范围构造函数std::vector::vector()构造一个容器，其中包含 first 到 last 范围内的尽可能多的元素。

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

int main(void) {

    std::vector<int> vec(5);

    for(int i = 0; i < vec.size(); ++i){
        vec[i] = i+1;
    }

    std::vector<int> vec2(vec.begin(),vec.end()); //范围构造

    for(int i = 0; i<vec2.size(); i++){
        std::cout<< vec2[i]<<std::endl;
    }

    return 0;
}
```

终端输出如下：

```bash
1
2
3
4
5
```

#### 4、复制构造构造一个容器，其中包含现有容器 x中存在的每个元素的副本

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

int main(void) {

    std::vector<int> vec(5);

    for(int i = 0; i < vec.size(); ++i){
        vec[i] = i+1;
    }

    std::vector<int> vec2(vec); //复制构造

    for(int i = 0; i<vec2.size(); i++){
        std::cout<< vec2[i]<<std::endl;
    }

    return 0;
}
```

输出如下：

```bash
1
2
3
4
5
```

#### 5、移动构造，使用move语义构造具有other内容的容器

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

int main(void) {

    std::vector<int> vec_1(5,100);
    std::cout<<"未开始移动构造前的输出如下:"<<std::endl;
    for(int i = 0; i < vec_1.size(); ++i){
        std::cout<<"vec_1:"<<vec_1[i]<<std::endl;
    }
    std::cout<<"开始移动构造的输出如下:"<<std::endl;
    std::vector<int> vec_2(move(vec_1)); //复制构造函数
    for(int i = 0; i < vec_1.size(); ++i){
        std::cout<<"vec_1:"<<vec_1[i]<<std::endl;
    }
    for(int i = 0; i < vec_2.size(); ++i){
        std::cout<<"vec_2:"<<vec_2[i]<<std::endl;
    }

    return 0;
}
```

终端输出如下：

```bash
未开始移动构造前的输出如下:
vec_1:100
vec_1:100
vec_1:100
vec_1:100
vec_1:100
开始移动构造后的输出如下:
vec_2:100
vec_2:100
vec_2:100
vec_2:100
vec_2:100
```

#### 6、从初始化列表构造一个容器

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

int main(void) {

    auto list = {5,4,3,2,1}; //使用了自动类型推导

    std::vector<int> vec(list);

    for(int i = 0; i < vec.size(); ++i){
        std::cout<<"vec:"<<vec[i]<<std::endl;
    }

    return 0;
}
```

终端输出如下：

```bash
vec:5
vec:4
vec:3
vec:2
vec:1
```

### 成员函数

#### 1、vector::assign  通过替换旧值为向量元素分配新值

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   /* Create empty vector */
   vector<int> v;
   /* create initializer list */
   auto il = {1, 2, 3, 4, 5};

   /* assign values from initializer list */
   v.assign(il);

  /* display vector elements */
  for (int i = 0; i < v.size(); ++i)
      cout << v[i] << endl;

   return 0;
}
```

终端输出如下：

```python
1
2
3
4
5
```

#### 2、vector()::at() 返回对向量中位置n处元素的引用

返回值：如果 n 是有效的向量索引，则从指定位置返回一个元素。

​    如果向量对象是常量，则返回常量引用，否则返回非常量引用。

异常：如果 n 无效索引 out_of_bound 抛出异常。

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   auto il = {1, 2, 3, 4, 5};
   vector<int> v(il);

   for (int i = 0; i < v.size(); ++i)
      cout << v.at(i) << endl;

   return 0;
}
```

终端输出如下：

```bash
1
2
3
4
5
```

#### 3、vector::back 返回对向量最后一个元素的引用

返回值：返回 vector 最后一个元素。

​    如果 vector 对象是常量，则方法返回常量引用，否则返回非常量引用。

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   auto il = {1, 2, 3, 4, 5};
   vector<int> v(il);

   cout << "Last element of vector = " << v.back() << endl;

   return 0;
}
```

终端输出如下：

```bash
Last element of vector = 5
```

#### 4、vector::begin 返回指向向量第一个元素的随机访问迭代器

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   auto il = {1, 2, 3, 4, 5};
   vector<int> v(il);

   for (auto it = v.begin(); it != v.end(); ++it)
      cout << *it << endl;

   return 0;
}
```

终端输出如下：

```bash
1
2
3
4
5
```

#### 5、vector::end 返回一个迭代器，它指向向量容器中的past-the-end 元素

past-the-end 元素是向量中最后一个元素。

返回值：返回vector中末尾元素的迭代器。

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   vector<int> v = {1, 2, 3, 4, 5};

   for (auto it = v.begin(); it != v.end(); ++it)
      cout << *it << endl;

   return 0;
}
```

终端输出如下：

```bash
1
2
3
4
5
```

####

#### 6、vector::capacity() 返回分配存储的大小，以元素表示

这个容量不一定等于向量的大小。它可以等于或大于向量大小。

向量大小的理论限制由成员max_size给出。

返回值：返回分配存储的大小，以向量可容纳的元素个数表示。

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   vector<int> v;

   for (int i = 0; i < 5; ++i)
      v.push_back(i + 1);

   cout << "Number of elements in vector = " << v.size() << endl;
   cout << "Capacity of vector           = " << v.capacity() << endl;

   return 0;
}
```

终端输出如下：

```bash
Number of elements in vector = 5
Capacity of vector           = 8
```

#### 7、vector::clear 通过从向量中删除所有元素并将向量的大小设置为零来销毁向量

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   auto ilist = {1, 2, 3, 4, 5};
   vector<int> v(ilist);

   cout << "Initial size of vector     = " << v.size() << endl;
   /* destroy vector */
   v.clear();
   cout << "Size of vector after clear = " << v.size() << endl;

   return 0;
}
```

终端输出如下：

```bash
Initial size of vector     = 5
Size of vector after clear = 0
```

#### 8、vector::empty() 测试向量是否为空

返回值：如果向量为空，则返回true，否则返回false

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   vector<int> v;

   if (v.empty())
      cout << "Vector v1 is empty" << endl;

   v.push_back(1);
   v.push_back(2);
   v.push_back(3);

   if (!v.empty())
      cout << "Vector v1 is not empty" << endl;

   return 0;
}
```

终端输出如下：

```bash
Vector v1 is empty
Vector v1 is not empty
```

#### 9、 vector::erase() 从向量中删除单个元素

返回值：返回一个随机访问迭代器。

异常：如果位置无效，则行为undefined

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
    vector<int> v = {1, 2, 3, 4, 5};

    cout << "Original vector" << endl;
    for (auto it = v.begin(); it != v.end(); ++it)
        cout << *it << endl;

    v.erase(v.begin());  //删除单个元素

    cout << "Modified vector" << endl;
    for (auto it = v.begin(); it != v.end(); ++it)
        cout << *it << endl;

    v.erase(v.begin(), v.begin() + 2);   //删除范围元素

    cout << "Modified vector" << endl;
    for (auto it = v.begin(); it != v.end(); ++it)
        cout << *it << endl;

    return 0;
}
```

终端输出如下：

```bash
Original vector
1
2
3
4
5
Modified vector
2
3
4
5
Modified vector
4
5
```

#### 10、vector::front() 返回对向量第一个元素的引用

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   vector<int> v = {1, 2, 3, 4, 5};

   cout << "First element of vector = " << v.front() << endl; //输出第一个元素

   return 0;
}
```

终端输出如下：

```bash
First element of vector = 1
```

#### 11、vector::pop_back() 从向量中删除最后一个元素并将向量的大小减小一

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   vector<int> v = {1, 2, 3, 4, 5};

   /* 删除3次末尾元素 */
   v.pop_back();
   v.pop_back();
   v.pop_back();

   for (int i = 0; i < v.size(); ++i)
      cout << v[i] << endl;

   return 0;
}
```

终端输出如下：

```bash
1
2
```

#### 12、vector::push_back() 在向量末尾插入新元素并将向量的大小增加一

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   vector<int> v;

   /* Insert 5 elements */
   for (int i = 0; i < 5; ++i)
      v.push_back(i + 1);

   for (int i = 0; i < v.size(); ++i)
      cout << v[i] << endl;

   return 0;
}
```

终端输出如下：

```bash
1
2
3
4
5
```

#### 13、vector::resize() 改变向量的大小

如果 n 小于当前大小，则销毁额外的元素。

如果 n 大于当前容器大小，则在向量末尾插入新元素。

如果指定了value值，那么新元素将使用value进行初始化。

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   vector<int> v;

   cout << "Initial vector size = " << v.size() << endl;

   v.resize(5, 10);
   cout << "Vector size after resize = " << v.size() << endl;

   cout << "Vector contains following elements" << endl;
   for (int i = 0; i < v.size(); ++i)
      cout << v[i] << endl;

   return 0;
}
```

终端输出如下：

```bash
Initial vector size = 0
Vector size after resize = 5
Vector contains following elements
10
10
10
10
10
```

#### 14、vector::swap() 交换两个向量的内容

时间复杂度：O(1)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
    vector<int> v1;
    vector<int> v2 = {1, 2, 3, 4, 5};

    v1.swap(v2);

    cout << "Vector v1 contains:" << endl;
    for (int i = 0; i < v1.size(); ++i)
        cout << v1[i] << endl;

    cout << "Vector v2 contains:" << endl;
    for (int i = 0; i < v2.size(); ++i)
        cout << v2[i] << endl;
   return 0;
}
```

终端输出如下：

```bash
Vector v1 contains:
1
2
3
4
5
Vector v2 contains:
```

### 非成员重载函数

#### 1、operator == 测试两个向量是否相等

返回值：如果两个向量相等，则返回true,否则返回false

时间复杂度：O(n)

代码如下：

```cpp
#include <iostream>
#include <vector>

using namespace std;

int main(void) {
   vector<int> v1;
   vector<int> v2;

   if (v1 == v2)
      cout << "v1 and v2 are equal" << endl;

   v1.resize(10, 100);

   if (!(v1 == v2))
      cout << "v1 and v2 are not equal" << endl;

   return 0;
}
```

终端输出如下：

```bash
v1 and v2 are equal
v1 and v2 are not equal
```

#### 2、operator != 测试两个向量是否相等

返回值：如果两个向量不相等，则返回true,否则返回false

时间复杂度：O(n)

#### 3、operator < 测试第一个向量是否小于其他向量

#### 4、operator <= 测试第一个向量是否小于或等于其他向量

#### 5、operator >测试第一个向量是否大于其他向量

#### 6、operator >= 测试第一个向量是否大于或等于其他向量

### 使用vector注意事项

- 如果你要表示的向量长度较长（需要为向量内部保存很多数），容易导致内存泄漏，而且效率会很低；

- Vector 作为函数的参数或者返回值时，需要注意它的写法：

```cpp
double Distance(vector<int>&a, vector<int>&b)
```

 其中的“&”绝对不能少！！！

### 排序

1. 使用reverse将元素翻转：需要头文件 `#include<algorithm>`

    `reverse(vec.begin(),vec.end());`将元素翻转，即逆序排列！

    (在vector中，如果一个函数中需要两个迭代器，一般后一个都不包含)

2. 使用 `sort` 排序：需要头文件 `#include<algorithm>`

    `sort(vec.begin(),vec.end());`(默认是按升序排列,即从小到大)。

    可以通过重写排序比较函数按照降序比较，如下：

    定义排序比较函数：

    ```cpp
    bool Comp(const int &a,const int &b)
    {
        return a>b;
    }
    ```

    调用时: `sort(vec.begin(),vec.end(),Comp)`，这样就降序排序

### 使用vector定义二维数组的两种方法

```cpp title='方法一'
#include <string.h>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;
 
 
int main()
{
    int N=5, M=6; 
    vector<vector<int> > obj(N); //定义二维动态数组大小5行 
    for(int i =0; i< obj.size(); i++)//动态二维数组为5行6列，值全为0 
    { 
        obj[i].resize(M); 
    } 
 
    for(int i=0; i< obj.size(); i++)//输出二维动态数组 
    {
        for(int j=0;j<obj[i].size();j++)
        {
            cout<<obj[i][j]<<" ";
        }
        cout<<"\n";
    }
    return 0;
}
```

```cpp title='方法二'
#include <string.h>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;
 
 
int main()
{
    int N=5, M=6; 
    vector<vector<int> > obj(N, vector<int>(M)); //定义二维动态数组5行6列 
 
    for(int i=0; i< obj.size(); i++)//输出二维动态数组 
    {
        for(int j=0;j<obj[i].size();j++)
        {
            cout<<obj[i][j]<<" ";
        }
        cout<<"\n";
    }
    return 0;
}
```

```cpp title='输出结果'
0 0 0 0 0 0 
0 0 0 0 0 0 
0 0 0 0 0 0 
0 0 0 0 0 0 
0 0 0 0 0 0 
```

### 参考致谢

- [C++ vector使用方法](https://www.w3cschool.cn/cpp/cpp-i6da2pq0.html)
- [C++ vector 容器浅析](https://www.runoob.com/w3cnote/cpp-vector-container-analysis.html)
