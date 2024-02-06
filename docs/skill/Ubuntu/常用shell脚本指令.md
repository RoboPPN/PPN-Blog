1. **shebang（脚本头）：** 在脚本文件的第一行，指定解释器。对于Bash脚本，通常是`#!/bin/bash`。

   ```bash
   #!/bin/bash
   ```

2. **变量赋值：** 使用`=`运算符为变量赋值。

   ```bash
   my_variable="Hello, World!"
   ```

3. **echo：** 用于输出文本到标准输出。

   ```bash
   echo "Hello, World!"
   ```

4. **读取用户输入：** 使用`read`命令从用户处接收输入。

   ```bash
   read -p "Enter your name: " username
   echo "Hello, $username!"
   ```

5. **条件语句（if语句）：** 用于根据条件执行不同的代码块。

   ```bash
   if [ "$variable" -eq 10 ]; then
       echo "The variable is 10."
   else
       echo "The variable is not 10."
   fi
   ```

6. **循环（for循环）：** 用于循环处理一系列数据。

   ```bash
   for i in {1..5}; do
       echo "Iteration $i"
   done
   ```

7. **函数定义：** 可以使用`function`关键字或者直接使用`() {}`来定义函数。

   ```bash
   my_function() {
       echo "This is a function."
   }
   ```

8. **文件和目录操作：**

   - 检查文件或目录是否存在：

     ```bash
     if [ -e "$file_path" ]; then
         echo "File exists."
     fi
     ```

   - 创建目录：

     ```bash
     mkdir my_directory
     ```

   - 复制文件：

     ```bash
     cp source_file destination
     ```

   - 移动/重命名文件：

     ```bash
     mv old_file new_location
     ```

   - 删除文件：

     ```bash
     rm file_to_delete
     ```

9. **管道和重定向：** 使用`|`将一个命令的输出传递给另一个命令，使用`>`和`>>`将输出重定向到文件。

   ```bash
   command1 | command2
   command > output.txt
   command >> append_output.txt
   ```
