## 如何使用mysql创建一个数据库

要在MySQL中创建一个新的数据库，你可以按照以下步骤进行操作：

1. 打开MySQL命令行界面：在终端或命令提示符中输入以下命令并按回车键：

   ```bash
   mysql -u root -p
   ```

   如果使用的是root用户，将要求你输入root用户的密码。

2. 成功连接到MySQL服务器后，你将看到MySQL的命令行提示符（类似于`mysql>`）。

3. 创建数据库：在MySQL命令行界面输入以下命令来创建一个新的数据库，将`database_name`替换为你想要的数据库名：

   ```bash
   CREATE DATABASE database_name;
   ```

   例如，要创建一个名为`mydatabase`的数据库，可以使用以下命令：

   ```bash
   CREATE DATABASE mydatabase;
   ```

4. 确认数据库创建成功：执行以下命令来列出所有已创建的数据库：

   ```bash
   SHOW DATABASES;
   ```

   这将显示所有已创建的数据库列表。确保你的新数据库出现在列表中。

5. 最后，你可以退出MySQL命令行界面：

   ```bash
   exit;
   ```

现在你已经成功创建了一个新的数据库。你可以在后续操作中使用该数据库，并在其中创建表和执行其他操作。

请注意，创建数据库通常需要root用户或具有适当权限的用户来执行。确保在管理数据库时小心操作，并为用户授予适当的权限，以确保数据库的安全性。

## ERROR 1045 (28000): Access denied for user 'agx-ppn'@'localhost' (using password: NO)

安装好后运行`mysql`和`sudo mysql`出现如下错误：

```bash
ERROR 1045 (28000): Access denied for user 'agx-ppn'@'localhost' (using password: NO)  #这是运行mysql的结果
```

and

```bash
ERROR 1045 (28000): Access denied for user 'agx-ppn'@'localhost' (using password: NO)  #这是运行sudo mysql的结果
```

问题分析：没有足够的权限来访问MySQL

问题解决方案：尝试使用具有更高权限的用户（例如root用户）进行连接，然后查看并修改其他用户的权限。

要使用具有更高权限的用户（如root用户）进行连接并查看和修改其他用户的权限，可以按照以下步骤进行操作：

1. 使用具有root权限的用户连接到MySQL服务器。在终端中输入以下命令：

   ```bash
   mysql -u root -p
   ```

   然后输入root用户的密码。

2. 成功连接后，你将看到MySQL的命令行提示符（类似于`mysql>`）。现在你可以执行SQL语句来查看和修改用户权限。

3. 查看用户权限：执行以下SQL语句，将`username`替换为你要查看的用户名：

   ```bash
   SHOW GRANTS FOR 'username'@'localhost';
   ```

   这将显示指定用户在本地主机上的权限列表。你可以检查这些权限是否满足你的需求。

4. 修改用户权限：如果你确定用户需要更高的权限，请使用以下SQL语句修改用户权限，将`username`替换为要修改的用户名，`localhost`替换为主机名或IP地址，`privileges`替换为你要赋予的具体权限：

   ```bash
   GRANT privileges ON database_name.* TO 'username'@'localhost';
   ```

   其中，`database_name`是你要授予访问权限的数据库名，`privileges`是具体的权限，例如`SELECT`、`INSERT`、`UPDATE`等。如果要授予所有权限，可以使用`ALL PRIVILEGES`。

5. 修改完成后，使用以下命令刷新权限：

   ```bash
   FLUSH PRIVILEGES;
   ```

6. 最后，你可以退出MySQL命令行界面：

   ```bash
   exit;
   ```

请注意，修改用户权限需要root用户或具有修改用户权限的用户来执行。确保小心管理权限，并仅为用户授予所需的最低权限，以确保数据库的安全性。

## DELETE  FROM charge_info Error Code: 1175

当我在MySQL Workbench运行

```sql
DELETE FROM charge_info;
```

这个操作时，出现了代号1175类型报错。

代号1175报错如下：

```bash
DELETE  FROM charge_info Error Code: 1175. You are using safe update mode and you tried to update a table without a WHERE that uses a KEY column.  To disable safe mode, toggle the option in Preferences -> SQL Editor and reconnect. 
```

这个报错是由于MySQL的"Safe Update Mode"引起的，它是一种安全机制，用于防止在不带有 WHERE 子句的情况下更新或删除表中的数据，以避免意外的数据丢失。在这种模式下，更新或删除操作必须使用带有主键（或唯一键）的 WHERE 子句，以确保只影响到特定的行。为了解决这个问题，可以按照以下几种方式之一操作：

1、使用带有 WHERE 子句的 DELETE 操作：

为了符合 "Safe Update Mode" 的要求，你可以使用带有主键的 WHERE 子句，以便只删除特定的行。例如，假设 charge_info_id 是主键：

```sql
DELETE FROM charge_info WHERE charge_info_id = your_id_value;
```

在这里，your_id_value 需要替换为你想要删除的行的 charge_info_id 值。

2、暂时禁用 "Safe Update Mode"：

如果你确定你的操作是安全的，你可以暂时禁用 "Safe Update Mode"，然后再执行 DELETE 操作。在执行之前，请确保你清楚自己在做什么，以避免意外数据丢失。

如果你使用 MySQL Workbench 等工具，你可以按照报错信息中提到的方式，在 "Preferences -> SQL Editor" 中关闭 "Safe Updates" 选项。然后重新连接数据库，即可执行 DELETE 操作。

请在操作之前备份重要数据，以免发生意外情况。选择合适的解决方法取决于你的实际需求和安全性要求。

这个问题是在使用MySQL Workbench时出现的，在终端使用`mysql -u root -p`方式操作

## 使用mysql创建表格，出现No database selected

mysql> CREATE TABLE students (    -> student_id INT UNSIGNED,    -> name VARCHAR(30) ,    -> sex CHAR(1),    -> birth DATE,    -> PRIMARY KEY(student_id)    -> ); ERROR 1046 (3D000): No database selected 这是什么原因？

这个错误提示表明在执行创建表的操作之前，没有选择任何数据库。在MySQL中，创建表需要在一个特定的数据库中进行操作。

要解决这个问题，你需要先选择一个已经存在的数据库，然后再执行创建表的操作。你可以按照以下步骤进行操作：

1. 列出当前存在的数据库：在MySQL命令行界面输入以下命令：

   ```bash
   SHOW DATABASES;
   ```

   这将列出所有已经创建的数据库。

2. 选择一个数据库：从列出的数据库中选择一个你想要使用的数据库，使用以下命令选择数据库，将`database_name`替换为你要使用的数据库名：

   ```bash
   USE database_name;
   ```

   例如，如果你要选择名为`mydatabase`的数据库，可以使用以下命令：

   ```bash
   USE mydatabase;
   ```

3. 确保你已经成功选择了数据库后，再执行创建表的操作。在MySQL命令行界面输入你的创建表的SQL语句：

   ```bash
   CREATE TABLE students (
   student_id INT UNSIGNED,
   name VARCHAR(30),
   sex CHAR(1),
   birth DATE,
   PRIMARY KEY(student_id)
   );
   ```

   这将在你选择的数据库中创建名为`students`的表。

请确保在执行创建表之前，先选择了一个存在的数据库。这样就能解决错误1046 "No database selected"的问题。

## 用于编写mysql的IDE

有一些专门用于编写和管理MySQL的集成开发环境（IDE）。以下是一些常用的MySQL IDE：

1. MySQL Workbench：MySQL官方提供的跨平台IDE，支持数据库建模、SQL开发、数据库管理和可视化等功能。
2. Navicat for MySQL：功能强大的数据库管理工具，提供了直观的界面、SQL编辑器、数据同步和导入导出等功能。
3. DBeaver：一款免费开源的通用数据库工具，支持多种数据库，包括MySQL。它具有强大的查询编辑器、模型设计工具和数据导入导出功能。
4. SQLyog：提供了可视化的数据库管理和SQL开发工具，具有直观的用户界面、强大的查询构建器和数据库同步功能。
5. HeidiSQL：适用于Windows的免费开源MySQL管理工具，具有用户友好的界面、SQL编辑器和数据浏览器。

这些工具提供了更便捷的方式来编写和管理MySQL数据库，它们通常具有语法高亮、代码补全、查询执行计划、数据可视化等功能，提高了开发和管理的效率。

## 数据库存储中文方法

在创建数据库时加上DEFAULT CHARACTER SET utf8，例如：

```sql
create database if not exists maverick_data DEFAULT CHARACTER SET utf8;
```
