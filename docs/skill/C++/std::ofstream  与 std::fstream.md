## std::ofstream  与 std::fstream

`std::ofstream` 和 `std::fstream` 都是 C++ 标准库中的类，用于文件操作。它们都属于 `<fstream>` 头文件提供的类，可以用于打开、读取、写入文件数据。

1. **std::ofstream**：

   `std::ofstream` 是 C++ 标准库中的一个类，用于进行文件输出操作。它继承自 `std::ostream`，因此可以使用所有输出操作符（如 `<<`）来写入数据到文件。

   示例用法：

   ```
   cppCopy code#include <iostream>
   #include <fstream>
   
   int main() {
       std::ofstream outFile("output.txt"); // 打开文件以进行输出
   
       if (outFile.is_open()) {
           outFile << "Hello, world!" << std::endl;
           outFile << "This is an example of ofstream." << std::endl;
   
           outFile.close(); // 关闭文件
       } else {
           std::cout << "Failed to open file for writing." << std::endl;
       }
   
       return 0;
   }
   ```

2. **std::fstream**：

   `std::fstream` 是 C++ 标准库中的一个类，用于进行文件读写操作。它继承自 `std::iostream`，既可以用于输入也可以用于输出，因此可以使用所有输入输出操作符来进行文件读写。

   示例用法：

   ```
   cppCopy code#include <iostream>
   #include <fstream>
   
   int main() {
       std::fstream file("data.txt", std::ios::in | std::ios::out); // 打开文件以进行读写操作
   
       if (file.is_open()) {
           std::string line;
           while (std::getline(file, line)) {
               std::cout << line << std::endl;
           }
   
           file << "This line is added using fstream." << std::endl;
   
           file.close(); // 关闭文件
       } else {
           std::cout << "Failed to open file." << std::endl;
       }
   
       return 0;
   }
   ```

总结来说，`std::ofstream` 主要用于文件输出操作，而 `std::fstream` 则可以用于文件读写操作。在使用时，可以根据具体的需求来选择合适的类。
