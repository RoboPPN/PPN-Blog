## 将数据从ubuntu20.04迁移至ubuntu18.04

我的mysql数据库运行在本地计算机上，如何将它迁移至其他计算机内？

要将MySQL数据库从一台计算机迁移到另一台计算机，可以按照以下步骤进行操作：

1. 备份数据库：首先，在原始计算机上创建数据库备份。可以使用MySQL的命令行工具或图形用户界面（如phpMyAdmin）执行备份操作。使用以下命令在命令行中备份数据库：

```bash
mysqldump -u [用户名] -p [数据库名] > backup.sql
```

这将整个数据库备份到名为backup.sql的文件中。

2. 将备份文件传输至目标计算机：将备份文件backup.sql从原始计算机传输到目标计算机。

3. 在目标计算机上安装MySQL：确保目标计算机上安装了MySQL数据库服务器。

4. 恢复数据库备份：使用MySQL的命令行工具或图形用户界面，在目标计算机上执行数据库恢复操作。使用以下命令在命令行中恢复数据库：

```bash
mysql -u [用户名] -p [数据库名] < backup.sql
```

这将使用备份文件backup.sql恢复数据库。

在运行上述命令行进行迁移时，ubuntu18.04的系统出现如下`ERROR`:

```bash
Unknown collation: 'utf8mb4_0900_ai_ci'
```

这是因为我备份的数据库`backup.sql`文件中有如下表格：

```sql
CREATE TABLE `xxxxx` (
  `xxx` int NOT NULL AUTO_INCREMENT,
  `xxx` int DEFAULT NULL,
  `xxx` varchar(30) DEFAULT NULL,
  `xxx` varchar(15) DEFAULT NULL,
  `xxx` tinyint(1) DEFAULT NULL,
) ENGINE=InnoDB AUTO_INCREMENT=28 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
```

这表示该表的存储引擎为InnoDB，自增起始值为21，并使用了utf8mb4字符集和utf8mb4_0900_ai_ci校对规则。

这个错误是由于在导入备份文件时，目标系统上的MySQL版本不支持utf8mb4_0900_ai_ci校对规则导致的。该校对规则在MySQL 5.7.7及更高版本中引入，而Ubuntu 18.04可能使用的是较早版本的MySQL。

为了解决这个问题，可以尝试以下两种方法：

- 修改备份文件：打开备份文件`backup.sql`，使用文本编辑器查找并替换所有出现的`utf8mb4_0900_ai_ci`为一个较早版本的校对规则，例如`utf8mb4_general_ci`。确保替换所有出现的实例。然后保存文件并重新导入。

- 修改MySQL配置：在目标系统上的MySQL配置文件中，通常是`my.cnf`文件，添加以下行来设置默认字符集和校对规则：

   ```
   iniCopy code[mysqld]
   character-set-server=utf8mb4
   collation-server=utf8mb4_general_ci
   ```

   保存文件并重启MySQL服务。

无论哪种方法，都需要确保备份文件中的数据与目标系统的MySQL版本兼容。如果备份文件包含特定于较新MySQL版本的功能或语法，可能需要根据目标系统的MySQL版本进行进一步的修改和调整。

5. 配置数据库连接和权限：在目标计算机上，根据需要，配置数据库连接和访问权限。确保你具有连接到数据库的正确凭据，并根据需要分配和管理用户权限。

完成上述步骤后，你的MySQL数据库就会从原始计算机成功迁移到目标计算机上。

请注意，迁移过程可能因操作系统和MySQL版本的不同而有所差异。确保在迁移过程中遵循MySQL文档和相关指南，并备份重要数据以防止意外数据丢失。
