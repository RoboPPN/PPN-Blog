## MySQL 数据类型

### 整数类型

|      类型      | 大小（字节） | 有符号数取值范围                | 无符号数取值范围   | 说明     |
| :------------: | ------------ | ------------------------------- | ------------------ | -------- |
|    TINYINT     | 1            | (-128, 127)                     | (0, 255)           | 超小整数 |
|    SMALLINT    | 2            | (-32 768, 32 767)               | (0, 65 535)        | 小整数   |
|   MEDIUMINT    | 3            | (-8 388 608, 8 388 607)         | (0, 16 777 215)    | 中等整数 |
| INT 或 INTEGER | 4            | (-2 147 483 648, 2 147 483 647) | (0, 4 294 967 295) | 整数     |
|     BIGINT     | 8            | (-263, 263-1)                   | (0, 264-1)         | 大整数   |

| 类型 | 说明                                                         |
| ---- | ------------------------------------------------------------ |
| BOOL | 布尔类型，只有 true 和 false 两个有效值；零值被认为是 false，非零值被认为是 true。  注意，MySQL 并不真正支持 BOOL 类型，BOOL 是 TINYINT(1) 的别名。 |

### 字符串类型

|            类型            | 说明                                                         |
| :------------------------: | ------------------------------------------------------------ |
|         CHAR(size)         | 用于表示固定长度的字符串，该字符串可以包含数字、字母和特殊字符。size 的大小可以是从 0 到 255 个字符，默认值为 1。 |
|       VARCHAR(size)        | 用于表示可变长度的字符串，该字符串可以包含数字、字母和特殊字符。size 的大小可以是从 0 到 65535 个字符。 |
|          TINYTEXT          | 表示一个最大长度为 255（28-1）的字符串文本。                 |
|         TEXT(size)         | 表示一个最大长度为 65,535（216-1）的字符串文本，也即 64KB。  |
|         MEDIUMTEXT         | 表示一个最大长度为 16,777,215（224-1）的字符串文本，也即 16MB。 |
|          LONGTEXT          | 表示一个最大长度为 4,294,967,295（232-1）的字符串文本，也即 4GB。 |
| ENUM(val1, val2, val3,...) | 字符串枚举类型，最多可以包含 65,535 个枚举值。插入的数据必须位于列表中，并且只能命中其中一个值；如果不在，将插入一个空值。 |
| SET( val1,val2,val3,....)  | 字符串集合类型，最多可以列出 64 个值。插入的数据可以命中其中的一个或者多个值，如果没有命中，将插入一个空值。 |

说明：ENUM 类型相当于单选题，SET 类型相当于多选题。

### 日期时间类型

| 类型           | 说明                                                         |
| -------------- | ------------------------------------------------------------ |
| DATE           | 日期类型，格式为 YYYY-MM-DD，取值范围从 '1000-01-01' 到 '9999-12-31'。 |
| DATETIME(fsp)  | 日期和时间类型，格式为 YYYY-MM-DD hh:mm:ss，取值范围从 '1000-01-01 00:00:00' 到 '9999-12-31 23:59:59'。 |
| TIMESTAMP(fsp) | 时间戳类型，它存储的值为从 Unix 纪元（'1970-01-01 00:00:00' UTC）到现在的秒数。TIMESTAMP 的格式为 YYYY-MM-DD hh:mm:ss，取值范围从 '1970-01-01 00:00:01' UTC 到 '2038-01-09 03:14:07' UTC。 |
| TIME(fsp)      | 时间类型，格式为 hh:mm:ss，取值范围从 '-838:59:59' 到 '838:59:59'。 |
| YEAR           | 四位数字的年份格式，允许使用从 1901 到 2155 之间的四位数字的年份。此外，还有一个特殊的取值，就是 0000。 |

### 二进制类型

|      类型       | 说明                                                         |
| :-------------: | ------------------------------------------------------------ |
|    BIT(size)    | 二进制位（Bit）类型，位数由 size 参数指定；size 的大小从 1 到 64，默认值为 1。 |
|  BINARY(Size)   | 等价于 CHAR() 类型，但是存储的是二进制形式的字节串。size 参数以字节（Byte）为单位指定列的长度，默认值为1。 |
| VARBINARY(Size) | 等价于 VARCHAR() 类型，但是存储的是二进制形式的字节串。size 参数以字节（Byte）为单位指定列的最大长度。 |
|    TINYBLOB     | 存储较小的二进制数据，最多可容纳 255 (28-1)个字节。          |
|   BLOB(size)    | 用来储存二进制数据，最多可以容纳 65,535（216-1）个字节，也即 64KB。 |
|   MEDIUMBLOB    | 存储中等大小的二进制数据，最多可以容纳 16,777,215（224-1）字节，也即 16MB。 |
|    LONGBLOB     | 存储较大的二进制数据，最多可容纳 42,94,967,295（232-1）字节，也即 4GB。 |

说明：BLOB 是 Binary Large Objects 的缩写，译为“大型二进制对象”，也即二进制数据块。

## CREATE DATABASE：创建数据库

CREATE DATABASE 语句的基本语法如下：

```sql
CREATE DATABASE <DatabaseName>;
```

DatabaseName 为数据库名字，它的名字必须是唯一的，不能和其它数据库重名。

## SHOW DATABASES:查看当前创建的数据库

## DROP DATABASE：删除数据库

DROP DATABASE 语句的基本语法如下：

```sql
DROP DATABASE <DatabaseName>;
```

DatabaseName 表示要删除的数据库。

## USE：选择数据库

USE 语句的基本语法如下：

```sql
USE <DatabaseName>;
```

DatabaseName 表示要选择的数据库名称，它必须是存在的。

## CREATE TABLE：创建表

- CREATE TABLE 语句的基本语法如下：

  ```sql
  CREATE TABLE table_name(
     column1 datatype,
     column2 datatype,
     column3 datatype,
     .....
     columnN datatype,
     PRIMARY KEY( one or more columns )
  );
  ```

  CREATE TABLE 是 SQL 命令，告诉数据库你想创建一个新的表，它后面紧跟的 table_name 是表的名字。然后在括号中定义表的列，以及每一列的类型，稍后会有更加清晰明了的示例。

  PRIMARY KEY 关键字用来指明表的主键。

## **DESC** 查看表结构

**DESC** 语句的基本语法如下：

```sql
DESC <TableName>;
```

TableName 表示要查看的表名称。

## DROP TABLE：删除表

DROP TABLE 语句的基本语法如下：

```sql
DROP TABLE <table_name>;
```

table_name 表示要删除的数据表的名字。

## SQL重命名表

我们以一个名为 website 的表为例，由于某种原因，我们希望将其更名为 tb_website。可以使用以下两种方式来修改表名：

```sql
ALTER TABLE website RENAME TO tb_website;
```

或者

```sql
RENAME website TO tb_website;
```

执行以上命令，website 表将被重命名为 tb_website。

## INSERT INTO语句：插入数据

INSERT INTO 语句有两种基本的用法。

1. 按指定的列插入数据，语法如下：

   ```sql
   INSERT INTO table_name (column1, column2, column3,...columnN) 
   VALUES (value1, value2, value3,...valueN);
   ```

column1, column2, column3,...columnN 表示要插入数据的列名，value1, value2, value3,...valueN 表示每列对应的值。

2. 为所有列插入数据，语法如下：

   ```sql
   INSERT INTO table_name VALUES (value1,value2,value3,...valueN);
   ```

为表中所有列添加数据时，可以不在 SQL 语句中指明列的名称，但是，请您确保插入的值的顺序和表中各列的顺序相同。

## 使用一个表的数据填充另一个表

使用 SELECT 语句可以从另一个表中选取一组数据，这组数据可以使用 INSERT INTO 语句填充到当前的表。但前提是，另一个表中必须有一组字段和当前表的字段是匹配的。

填充的语法如下：

```sql
INSERT INTO first_table_name [(column1, column2, ... columnN)]    SELECT column1, column2, ...columnN    FROM second_table_name    [WHERE condition];
```

## SELECT语句：选取数据SELECT

SELECT 语句的基本语法如下：

```sql
SELECT column1, column2, columnN
FROM table_name
WHERE conditions;
```

column1, column2, columnN 表示选取的列，conditions 表示筛选条件，只有满足条件的数据才会被选取。

WHERE 子句是可选的，您可以不写，此时 SELECT 语句将变成下面的形式：

```sql
SELECT column1, column2, columnN FROM table_name;
```

不使用 WHERE 子句意味着没有筛选条件，此时表中的所有数据都将被选取。

此外，如果您希望选取所有的列，那么可以使用`*`代替所有列名，语法如下：

```sql
SELECT * FROM table_name;
```

**SELECT** 子句

SELECT 可以结合下面的子句一起使用：

- WHERE 子句：用来指明筛选条件，只有满足条件的数据才会被选取。
- ORDER BY 子句：按照某个字段对结果集进行排序。
- GROUP BY 子句：结合聚合函数，根据一个或多个列对结果集进行分组。
- HAVING 子句：通常和 GROUP BY 子句联合使用，用来过滤由 GROUP BY 子句返回的结果集。

## WHERE子句：指定查询条件

使用 SQL 从单个表或者多表联合查询数据时，可以使用 **WHERE** 子句指定查询条件。当给定查询条件时，只有满足条件的数据才会被返回。建议您使用 WHERE 子句来过滤记录，以获取必要的结果集。

WHERE 子句不仅可以用在 SELECT 语句中，还可以用在 UPDATE、DELETE 等语句中。

WHERE 子句用于 SELECT 语句时的基本语法如下：

```sql
SELECT column1, column2, columnN
FROM table_name
WHERE condition
```

可以在 condition 条件中使用 >、<、= 等比较运算符，或者使用 AND、OR 等逻辑运算符来指定多个条件，或者使用 LIKE、NOT LIKE 等进行模糊匹配。

现在要查询日访问量（uv）大于 800 万的网站，并且只返回 id、name、url 和 uv 四个字段，代码如下：

```sql
SELECT id, name, url, uv FROM websiteWHERE uv > 800;
```

再如，查找日流量大于 500 万，并且名字里面包含字母 o 的网站，代码如下：

```sql
SELECT id, name, url, uv FROM websiteWHERE uv > 500 AND name LIKE '%o%';
```

## AND和OR运算符

SQL 中的 AND 和 OR 运算符用来连接多个查询条件，以缩小返回的结果集，它们被称为连接符。

**AND 运算符**

AND 运算符用于连接 WHERE 子句中的多个查询条件，只有当这些查询条件都被满足时，数据行（记录）才会被选取。

WHERE 子句中 AND 运算符的基本语法如下：

```sql
SELECT column1, column2, columnN
FROM table_name
WHERE [condition1] AND [condition2]...AND [conditionN];
```

您可以使用 AND 运算符连接 N 个条件，只有当这些条件都被满足时，SQL 语句才会奏效。

**OR 运算符**

OR 运算符用于连接 WHERE 子句中的多个查询条件，只要满足其中一个条件，数据行（记录）就能被选取。

WHERE 子句中 OR 运算符的基本语法如下：

```sql
SELECT column1, column2, columnN
FROM table_name
WHERE [condition1] OR [condition2]...OR [conditionN]
```

您可以使用 OR 运算符连接 N 个条件，只要满足其中一个条件，SQL 语句就奏效。

## UPDATE语句：修改数据

SQL **UPDATE** 语句用于修改数据表中现有的记录（数据行）。UPDATE 通常和 WHERE 子句一起使用，用以筛选满足条件的记录；如果不使用 WHERE 子句，那么表中所有的记录都将被修改，这往往不是我们期望的。

带有 WHERE 子句的 UPDATE 命令的基本语法如下：

```sql
UPDATE table_name
SET column1 = value1, column2 = value2...., columnN = valueN
WHERE [condition];
```

您可以使用 AND 或者 OR 运算符组合多个条件。

现在有一个包含如下记录的 web 表：

```
+----+----------------+----------------------------+-----+-------+---------+---------+
| id | name           | url                        | age | alexa | uv      | country |
+----+----------------+----------------------------+-----+-------+---------+---------+
|  1 | 百度           | https://www.baidu.com/     |  21 |     4 |  5010.5 | CN      |
|  2 | 淘宝           | https://www.taobao.com/    |  17 |     8 | 3996.75 | CN      |
|  3 | C语言中文网    | http://c.biancheng.net/    |  12 |  7923 |   11.62 | CN      |
|  4 | Google         | https://www.google.com/    |  23 |     1 |   36474 | US      |
|  5 | GitHub         | https://github.com/        |  13 |    95 |   216.3 | US      |
|  6 | Stack Overflow | https://stackoverflow.com/ |  16 |    48 |   592.2 | US      |
|  7 | Yandex         | http://www.yandex.ru/      |  11 |    53 |  591.82 | RU      |
|  8 | VK             | https://vk.com/            |  23 |    23 |    1206 | RU      |
+----+----------------+----------------------------+-----+-------+---------+---------+
```

以下 SQL 语句将更新表中 id 为 6 的网站的名字（name）：

```
UPDATE websiteSET name = 'stack-overflow'WHERE id = 6;
```

执行完该语句，website 表的记录如下：

```
+----+----------------+----------------------------+-----+-------+---------+---------+
| id | name           | url                        | age | alexa | uv      | country |
+----+----------------+----------------------------+-----+-------+---------+---------+
|  1 | 百度           | https://www.baidu.com/     |  21 |     4 |  5010.5 | CN      |
|  2 | 淘宝           | https://www.taobao.com/    |  17 |     8 | 3996.75 | CN      |
|  3 | C语言中文网    | http://c.biancheng.net/    |  12 |  7923 |   11.62 | CN      |
|  4 | Google         | https://www.google.com/    |  23 |     1 |   36474 | US      |
|  5 | GitHub         | https://github.com/        |  13 |    95 |   216.3 | US      |
|  6 | stack-overflow | https://stackoverflow.com/ |  16 |    48 |   592.2 | US      |
|  7 | Yandex         | http://www.yandex.ru/      |  11 |    53 |  591.82 | RU      |
|  8 | VK             | https://vk.com/            |  23 |    23 |    1206 | RU      |
+----+----------------+----------------------------+-----+-------+---------+---------+
```

如果您要修改表中所有记录的 age 和 country 值，则只需要 UPTATE 命令，不需要使用 WHERE 子句，请看下面的代码：

```
UPDATE websiteSET age = 20, country = 'CN';
```

执行完该语句，CUSTOMERS 表的记录如下：

```
+----+----------------+----------------------------+-----+-------+---------+---------+
| id | name           | url                        | age | alexa | uv      | country |
+----+----------------+----------------------------+-----+-------+---------+---------+
|  1 | 百度           | https://www.baidu.com/     |  20 |     4 |  5010.5 | CN      |
|  2 | 淘宝           | https://www.taobao.com/    |  20 |     8 | 3996.75 | CN      |
|  3 | C语言中文网    | http://c.biancheng.net/    |  20 |  7923 |   11.62 | CN      |
|  4 | Google         | https://www.google.com/    |  20 |     1 |   36474 | CN      |
|  5 | GitHub         | https://github.com/        |  20 |    95 |   216.3 | CN      |
|  6 | Stack Overflow | https://stackoverflow.com/ |  20 |    48 |   592.2 | CN      |
|  7 | Yandex         | http://www.yandex.ru/      |  20 |    53 |  591.82 | CN      |
|  8 | VK             | https://vk.com/            |  20 |    23 |    1206 | CN      |
+----+----------------+----------------------------+-----+-------+---------+---------+
```

## DELETE：删除数据

SQL DELETE 语句用于删除数据表中现有的记录。DELETE 命令通常和 WHERE 子句一起使用，用以删除满足条件的记录；如果不使用 WHERE 子句，那么表中所有的记录都将被删除。

带有 WHERE 子句的 DELETE 命令的基本语法如下：

```sql
DELETE FROM table_name
WHERE [condition];
```

你可以使用 AND 或者 OR 运算符连接多个条件。

如果您想删除 website 表中的所有记录，则无需使用 WHERE 子句，此时的 DELETE 命令如下所示：

```
DELETE FROM website;
```

执行完该语句，website 表将没有任何记录，也即变成一个空表。

## LIKE子句：模糊匹配

**LIKE** 子句用于在 WHERE 语句中进行模糊匹配，它会将给定的匹配模式和某个字段进行比较，匹配成功则选取，否则不选取。

LIKE 子句可以和通配符一起使用：

|           通配符           | 说明                                                         |
| :------------------------: | ------------------------------------------------------------ |
|        百分号（%）         | 代表零个、一个或者多个任意的字符。                           |
|        下划线（_）         | 代表单个字符或者数字。                                       |
|         [charlist]         | 字符列表中的任何单一字符。可以使用连字符（-）根据 ASCII 编码指定一个字符范围，例如：[0-9] 表示从 0 到 9 的任一数字；[a-z] 表示小写英文字母；[a-zA-Z] 表示英文字母，不区分大小写；[a-zA-Z0-9] 表示英文字母和阿拉伯数字。 |
| [^charlist] 或 [!charlist] | 不在字符列表中的任何单一字符。同上，也可以使用连字符（-）指定一个字符范围。 |

SQL LIKE 子句的基本语法格式如下：

```
SELECT FROM table_name WHERE column LIKE 'pattern'
```

pattern 表示给定的匹配模式。

您也可以使用 AND 或者 OR 运算符连接多个条件，例如：

```
SELECT FROM table_name
WHERE column1 LIKE 'pattern1' AND column2 LIKE 'pattern2'
```

**示例**

下面给出了一些示例，这些示例展示了 LIKE 可以使用的匹配模式：

| 序号 | 示例和说明                                                   |
| :--: | ------------------------------------------------------------ |
|  1   | WHERE uv LIKE '200%' 查找 uv 字段中以 200 开头的值。         |
|  2   | WHERE uv LIKE '%200%' 查找 uv 字段中包含 200 的值（200 可以在开头、末尾或者中间的任意位置）。 |
|  3   | WHERE uv LIKE '_00%' 查找 uv 字段中第二个和第三个字符都是 0 的值。 |
|  4   | WHERE uv LIKE '2_%_%' 查找 uv 字段中以 2 开头，且长度至少为 3 的任意值。 |
|  5   | WHERE uv LIKE '%2' 查找 uv 字段中以 2 结尾的值。             |
|  6   | WHERE name LIKE '%sh%' 查找 name 字段中包含 sh 的值。        |
|  7   | WHERE name LIKE '[xyz]' 查找 name 字段中至少包含 xyz 其中一个字符的值。 |
|  8   | WHERE name LIKE '[^a-e]' 查找 name 字段中不包含 a-e 中任何一个字符的值。 |

## TOP子句：限制返回数据的条数

**TOP** 子句用于限定要返回的记录的数据，可以是一个具体的数字，也可以是一个百分数。

对于拥有成千上万条记录的大型数据表来说，TOP 子句非常有用，它能够压缩结果集的大小，提高程序查询效率。

注意，并不是所有的数据库都支持 TOP 子句，有些数据库使用其它的等价语句来替代，例如：

- MySQL 使用 LIMIT 子句获取指定数量的记录；
- Oracle 使用 ROWNUM 子句获取指定数量的记录。

在MySQL 数据库中运行下面指令，它将获取 website 表中的前三条记录：

```
SELECT * FROM website LIMIT 3;
```

执行结果：

```
+----+-------------+-------------------------+-----+-------+---------+---------+
| id | name        | url                     | age | alexa | uv      | country |
+----+-------------+-------------------------+-----+-------+---------+---------+
|  1 | 百度        | https://www.baidu.com/  |  21 |     4 |  5010.5 | CN      |
|  2 | 淘宝        | https://www.taobao.com/ |  17 |     8 | 3996.75 | CN      |
|  3 | C语言中文网 | http://c.biancheng.net/ |  12 |  7923 |   11.62 | CN      |
+----+-------------+-------------------------+-----+-------+---------+---------+
```

## ORDER BY子句：排序

**ORDER BY** 子句用于根据一个或者多个字段对查询结果（结果集）进行排序，可以是降序，也可以是升序。默认情况下，大部分数据库将查询结果按照升序排序。

ORDER BY 子句的基本语法如下所示：

```sql
SELECT column_list
FROM table_name
[WHERE condition]
[ORDER BY column1, column2, .. columnN] [ASC | DESC];
```

您可以在 ORDER BY 子句中指定多个用于排序的字段，它们之间以逗号`,`分隔；但是，您应该确保这些字段都位于 column_list 中。

ASC 关键字表示升序，DESC 关键字表示降序；如果不写，大部分数据库默认为 ASC。

现在有一个包含如下记录的 website 表：

```
+----+----------------+----------------------------+-----+-------+---------+---------+
| id | name           | url                        | age | alexa | uv      | country |
+----+----------------+----------------------------+-----+-------+---------+---------+
|  1 | 百度           | https://www.baidu.com/     |  21 |     4 |  5010.5 | CN      |
|  2 | 淘宝           | https://www.taobao.com/    |  17 |     8 | 3996.75 | CN      |
|  3 | C语言中文网    | http://c.biancheng.net/    |  12 |  7923 |   11.62 | CN      |
|  4 | Google         | https://www.google.com/    |  23 |     1 |   36474 | US      |
|  5 | GitHub         | https://github.com/        |  13 |    95 |   216.3 | US      |
|  6 | Stack Overflow | https://stackoverflow.com/ |  16 |    48 |   592.2 | US      |
|  7 | Yandex         | http://www.yandex.ru/      |  11 |    53 |  591.82 | RU      |
|  8 | VK             | https://vk.com/            |  23 |    23 |    1206 | RU      |
+----+----------------+----------------------------+-----+-------+---------+---------+
```

下面的 SQL 语句将根据 age 和 uv 字段对结果集进行升序排序：

```
SELECT * FROM websiteORDER BY age, uv;
```

执行结果：

```
+----+----------------+----------------------------+-----+-------+---------+---------+
| id | name           | url                        | age | alexa | uv      | country |
+----+----------------+----------------------------+-----+-------+---------+---------+
|  7 | Yandex         | http://www.yandex.ru/      |  11 |    53 |  591.82 | RU      |
|  3 | C语言中文网    | http://c.biancheng.net/    |  12 |  7923 |   11.62 | CN      |
|  5 | GitHub         | https://github.com/        |  13 |    95 |   216.3 | US      |
|  6 | Stack Overflow | https://stackoverflow.com/ |  16 |    48 |   592.2 | US      |
|  2 | 淘宝           | https://www.taobao.com/    |  17 |     8 | 3996.75 | CN      |
|  1 | 百度           | https://www.baidu.com/     |  21 |     4 |  5010.5 | CN      |
|  8 | VK             | https://vk.com/            |  23 |    23 |    1206 | RU      |
|  4 | Google         | https://www.google.com/    |  23 |     1 |   36474 | US      |
+----+----------------+----------------------------+-----+-------+---------+---------+
```

再如，下面的 SQL 语句将根据 alexa 字段对结果集进行降序排序：

```
SELECT * FROM websiteORDER BY alexa DESC;
```

执行结果：

```
+----+----------------+----------------------------+-----+-------+---------+---------+
| id | name           | url                        | age | alexa | uv      | country |
+----+----------------+----------------------------+-----+-------+---------+---------+
|  3 | C语言中文网    | http://c.biancheng.net/    |  12 |  7923 |   11.62 | CN      |
|  5 | GitHub         | https://github.com/        |  13 |    95 |   216.3 | US      |
|  7 | Yandex         | http://www.yandex.ru/      |  11 |    53 |  591.82 | RU      |
|  6 | Stack Overflow | https://stackoverflow.com/ |  16 |    48 |   592.2 | US      |
|  8 | VK             | https://vk.com/            |  23 |    23 |    1206 | RU      |
|  2 | 淘宝           | https://www.taobao.com/    |  17 |     8 | 3996.75 | CN      |
|  1 | 百度           | https://www.baidu.com/     |  21 |     4 |  5010.5 | CN      |
|  4 | Google         | https://www.google.com/    |  23 |     1 |   36474 | US      |
+----+----------------+----------------------------+-----+-------+---------+---------+
```

## GROUP BY子句：分组

**GROUP BY** 子句用来根据指定的字段对结果集（选取的数据）进行分组，如果某些记录的指定字段具有相同的值，那么它们将被合并为一条数据。通俗地理解，GROUP BY 子句将根据指定的字段合并数据行。

借助 SQL 聚合函数，您可以对分组的数据进行再次加工，例如：

- SUM( ) 函数可以对指定字段的值进行求和；
- COUNT( ) 函数可以计算某个分组内数据的条数；
- AVG( ) 函数可以对指定字段的值求平均数。

GROUP BY 子句的基本语法如下：

```sql
SELECT column1, column2
FROM table_name
WHERE [ conditions ]
GROUP BY column1, column2
ORDER BY column1, column2
```

GROUP 子句使用说明：

- GROUP BY 子句需要和 SELECT 语句一起使用；
- 如果有 WHERE 子句，那么 WHERE 子句需要放在 GROUP BY 子句之前；
- 如果有 ORDER BY 子句，那么 ORDER BY 子句需要放在 GROUP 子句之后。

现在有包含如下记录的 website 表：

```
+----+----------------+----------------------------+-----+-------+---------+---------+
| id | name           | url                        | age | alexa | uv      | country |
+----+----------------+----------------------------+-----+-------+---------+---------+
|  1 | 百度           | https://www.baidu.com/     |  21 |     4 |  5010.5 | CN      |
|  2 | 淘宝           | https://www.taobao.com/    |  17 |     8 | 3996.75 | CN      |
|  3 | C语言中文网    | http://c.biancheng.net/    |  12 |  7923 |   11.62 | CN      |
|  4 | Google         | https://www.google.com/    |  23 |     1 |   36474 | US      |
|  5 | GitHub         | https://github.com/        |  13 |    95 |   216.3 | US      |
|  6 | Stack Overflow | https://stackoverflow.com/ |  16 |    48 |   592.2 | US      |
|  7 | Yandex         | http://www.yandex.ru/      |  11 |    53 |  591.82 | RU      |
|  8 | VK             | https://vk.com/            |  23 |    23 |    1206 | RU      |
+----+----------------+----------------------------+-----+-------+---------+---------+
```

如果您想知道每个国家网站的总访问量，那么 GROUP BY 子句的写法如下：

```
SELECT country, SUM(uv) AS total FROM websiteGROUP BY country;
```

[SQL AS](https://c.biancheng.net/sql/as.html) 关键字用来给字段起一个临时的别名，该别名只显示在结果集中，并不会更改原始表的字段名。执行结果：

```
+---------+--------------------+
| country | total              |
+---------+--------------------+
| CN      |  9018.869999885559 |
| RU      | 1797.8200073242188 |
| US      |  37282.50001525879 |
+---------+--------------------+
```

您可以使用 TRUNCATE() 函数让小数保留两位数字，具体写法如下：

```
SELECT country, TRUNCATE(SUM(uv), 2) AS total FROM websiteGROUP BY country;
```

执行结果：

```
+---------+----------+
| country | total    |
+---------+----------+
| CN      |  9018.86 |
| RU      |  1797.82 |
| US      | 37282.50 |
+---------+----------+
```

## DISTINCT：删除重复记录

**DISTINCT** 关键字需要和 SELECT 语句一起使用，用来删除结果集中所有重复的记录，仅保留唯一的一条记录。

数据表中有时候会有重复的记录，如果您只需要其中一条，就可以使用 DISTINCT 关键字。

DISTINCT 关键字的基本语法格式如下：

```sql
SELECT DISTINCT column1, column2,.....columnN
FROM table_name
WHERE [condition]
```

现在有包含如下记录的 website 表：

```
+----+----------------+----------------------------+-----+-------+---------+---------+
| id | name           | url                        | age | alexa | uv      | country |
+----+----------------+----------------------------+-----+-------+---------+---------+
|  1 | 百度           | https://www.baidu.com/     |  20 |     4 |  5010.5 | CN      |
|  2 | 淘宝           | https://www.taobao.com/    |  20 |     8 | 3996.75 | CN      |
|  3 | C语言中文网    | http://c.biancheng.net/    |  12 |  7923 |   11.62 | CN      |
|  4 | Google         | https://www.google.com/    |  23 |     1 |   36474 | US      |
|  5 | GitHub         | https://github.com/        |  15 |    95 |   216.3 | US      |
|  6 | Stack Overflow | https://stackoverflow.com/ |  15 |    48 |   592.2 | US      |
|  7 | Yandex         | http://www.yandex.ru/      |  11 |    53 |  591.82 | RU      |
|  8 | VK             | https://vk.com/            |  23 |    23 |    1206 | RU      |
+----+----------------+----------------------------+-----+-------+---------+---------+
```

下面的 SELECT 语法返回的记录中包含了重复的 age 和 country：

```
SELECT age, country FROM websiteORDER BY age;
```

执行结果：

```
+-----+---------+
| age | country |
+-----+---------+
|  11 | RU      |
|  12 | CN      |
|  15 | US      |
|  15 | US      |
|  20 | CN      |
|  20 | CN      |
|  23 | US      |
|  23 | RU      |
+-----+---------+
```

可以看到，age=15 和 country=US、age=20 和 country=CN 都重复出现了两次，这是因为原始表中就包含重复的记录。

现在，我们在 SELECT 语句中加入 DISTINCT 关键字，去除重复的记录：

```
SELECT DISTINCT age, country FROM websiteORDER BY age;
```

执行结果：

```
+-----+---------+
| age | country |
+-----+---------+
|  11 | RU      |
|  12 | CN      |
|  15 | US      |
|  20 | CN      |
|  23 | RU      |
|  23 | US      |
+-----+---------+
```

可以看到，age=15 和 country=US、age=20 和 country=CN 都只出现了一次，重复记录被去除。

## SQL约束简介

约束（Constraint）是指表的数据列必须强行遵守的规则，这些规则用于限制插入表中的数据类型，这样能够确保每份数据的准确定和可靠性。

约束可以是列级别，也可以是表级别；列级约束仅作用于某一列，而表级约束则作用于整张表。

下面是 SQL 常用的一些约束：

|                            约束                             | 说明                                                 |
| :---------------------------------------------------------: | ---------------------------------------------------- |
|    [NOT NULL](https://c.biancheng.net/sql/not-null.html)    | 非空约束，确保列中不能有 NULL 值。                   |
|     [DEFAULT](https://c.biancheng.net/sql/default.html)     | 默认约束，如果未指定值，那么列将提供默认值。         |
|      [UNIQUE](https://c.biancheng.net/sql/unique.html)      | 唯一约束，确保列中所有的值都不相同。                 |
| [PRIMARY Key](https://c.biancheng.net/sql/primary-key.html) | 主键，用来唯一标识数据库中的每一行/记录。            |
| [FOREIGN Key](https://c.biancheng.net/sql/foreign-key.html) | 外键，用来唯一标识任何其它数据库中表的行/记录。      |
|       [CHECK](https://c.biancheng.net/sql/check.html)       | 检查性约束，用于确保列中的所有值都必须满足某些条件。 |
|      [INDEX](https://c.biancheng.net/sql/indexes.html)      | 索引，用于快速从数据库中检索或者获取数据。           |

## NULL：空值

在 SQL 中，关键字 NULL 用来表示缺失的值，也即空值，或者没有值。NULL 值不等同于零值，也不等同于包含空格的字段，理解这一点非常重要。

表的字段默认允许存放 NULL 值，这意味着，您在插入记录或者更新记录时，可以不为该字段指定值，此时该字段将存储 NULL 值。

在查询结果中，空值将显示为空白或者 NULL，如下所示：

| sir_name | name  | marks |
| -------- | ----- | ----- |
| TYAGI    | SEEMA | NULL  |
| SINGH    | RAMAN | 5.5   |
| SHARMA   | AMAR  | NULL  |
| JAISWAL  | VICKY | 6.2   |

第一条和第三条记录的 MARKS 字段就出现了 NULL 值。

在创建数据表时可以指定某个字段是否允许为 NULL，基本语法如下：

```
CREATE TABLE website (    id      INT              NOT NULL   AUTO_INCREMENT,    name    VARCHAR(20)      NOT NULL,    url     VARCHAR(30),    age     TINYINT UNSIGNED NOT NULL,    alexa   INT UNSIGNED     NOT NULL,    uv      FLOAT                       DEFAULT '0',    country CHAR(3)          NOT NULL   DEFAULT '',    PRIMARY KEY (`id`));
```

NOT NULL 关键字表示不允许该字段为空值，在插入或者更新记录时必须为该字段指定一个具体的值。url 和 uv 两个字段没有使用 NOT NULL 关键字，这意味着它们保持默认，也即允许为 NULL。注意，uv 字段指定了默认值 0，如果不为该字段提供值，它的值将是 0；url 字段没有指定默认值，如果不为该字段提供值，它的值将是 NULL。

在选取数据时，NULL 值可能会导致一些问题，因为 NULL 值和其它任何值比较的结果都是未知的，所以包含 NULL 值的记录始终不能被筛选。

我们不能使用 =、<、> 等比较运算符来检测 NULL 值，而必须使用 IS NULL 或者 IS NOT NULL 关键字来检测 NULL 值。

现在有一个包含如下记录的 website 表：

```
+----+----------------+-------------------------+-----+-------+---------+---------+
| id | name           | url                     | age | alexa | uv      | country |
+----+----------------+-------------------------+-----+-------+---------+---------+
|  1 | 百度           | https://www.baidu.com/  |  20 |     4 |  5010.5 | CN      |
|  2 | 淘宝           | NULL                    |  20 |     8 | 3996.75 | CN      |
|  3 | C语言中文网    | http://c.biancheng.net/ |  12 |  7923 |   11.62 | CN      |
|  4 | Google         | https://www.google.com/ |  23 |     1 |   36474 | US      |
|  5 | GitHub         | https://github.com/     |  15 |    95 |   216.3 | US      |
|  6 | Stack Overflow | NULL                    |  15 |    48 |   592.2 | US      |
|  7 | Yandex         | http://www.yandex.ru/   |  11 |    53 |  591.82 | RU      |
|  8 | VK             | https://vk.com/         |  23 |    23 |    1206 | RU      |
+----+----------------+-------------------------+-----+-------+---------+---------+
```

下面是 IS NOT NULL 关键字的用法：

```
SELECT  id, name, age, uv, countryFROM websiteWHERE url IS NOT NULL;
```

执行结果：

```
+----+-------------+-----+--------+---------+
| id | name        | age | uv     | country |
+----+-------------+-----+--------+---------+
|  1 | 百度        |  20 | 5010.5 | CN      |
|  3 | C语言中文网 |  12 |  11.62 | CN      |
|  4 | Google      |  23 |  36474 | US      |
|  5 | GitHub      |  15 |  216.3 | US      |
|  7 | Yandex      |  11 | 591.82 | RU      |
|  8 | VK          |  23 |   1206 | RU      |
+----+-------------+-----+--------+---------+
```

## NOT NULL：非空约束

默认情况下，表的字段可以包含 NULL 值，如果您不希望某个字段出现 NULL 值，那么可以在该字段上添加 NOT NULL 约束（非空约束），此时就必须给该字段指定一个具体的值，不能留空。

注意，NULL 不等于没有数据，而是表示数据是未知的。

下面的 SQL 语句将创建一个名为 website 的新表，该表包含七个字段，其中五个字段不接受 NULL 值，它们分别是 id、name、age、alexa 和 country：

```
纯文本复制
CREATE TABLE website (    id      INT              NOT NULL   AUTO_INCREMENT,    name    VARCHAR(20)      NOT NULL,    url     VARCHAR(30),    age     TINYINT UNSIGNED NOT NULL,    alexa   INT UNSIGNED     NOT NULL,    uv      FLOAT                       DEFAULT '0',    country CHAR(3)          NOT NULL   DEFAULT '',    PRIMARY KEY (`id`));
```

如果已经创建了 website 表，也可以使用 ALTER TABLE 语句将 NOT NULL 约束添加到某个字段，例如：

```
ALTER TABLE websiteMODIFY url VARCHAR(30) NOT NULL;
```

## DEFAULT：默认约束

DEFAULT 约束用于给字段指定一个默认值，当使用 INSERT INTO 语句向表中插入数据时，如果没有为该字段提供具体的值，那么就使用这个默认值。

下面的 SQL 语句将创建一个名为 website 的新表，该表包含五个字段，其中 uv 和 country 字段拥有 DEFAULT 约束，默认值分别为 10.0 和空字符串`''`。如果 INSERT INTO 语句不为 uv 和 country 字段提供值，那么这两个字段将使用默认值 10.0 和空字符串`''`。

```
CREATE TABLE website (
    id      INT              NOT NULL   AUTO_INCREMENT,
    name    VARCHAR(20)      NOT NULL,
    url     VARCHAR(30),
    age     TINYINT UNSIGNED NOT NULL,
    alexa   INT UNSIGNED     NOT NULL,
    uv      FLOAT                       DEFAULT '10.0',
    country CHAR(3)          NOT NULL   DEFAULT '',
    PRIMARY KEY (`id`)
);
```

如果已经创建了 website 表，则可以使用 ALTER TABLE 语句将 DEFAULT 约束添加到 uv 字段，如下所示：

```
ALTER TABLE website MODIFY uv FLOAT DEFAULT '10.0';
```

**删除默认约束**

借助 ALTER TABLE 语句也可以删除默认约束，如下所示：

```
ALTER TABLE website ALTER COLUMN uv DROP DEFAULT;
```

## UNIQUE：唯一约束

SQL UNIQUE 约束也称“唯一约束”，设置了 UNIQUE 约束的字段，每条记录的值都必须是唯一的，因此 UNIQUE 约束可以防止两条记录在某个字段上出现重复值。例如在 CUSTOMERS 表中，要防止两个或者多个顾客出现相同的姓名。

UNIQUE 可以约束表的一个字段，也可以约束多个字段。此外，设置了 UNIQUE 约束的字段可以出现 NULL 值。

**UNIQUE 和 PRIMARY KEY 的区别**

- UNIQUE（唯一约束）和 PRIMARY KEY（主键）非常相似，但是 UNIQUE 允许字段中出现一次 NULL 值，而 PRIMARY KEY 不允许出现 NULL 值，因为可以认为：

  PRIMARY KEY = UNIQUE + NOT NULL

- 一张表可以包含多个 UNIQUE 字段，但是只能有一个主键。

下面的 SQL 语句将创建一个名为 website 的新表，该表包含七个字段，其中 alexa 被设置为 UNIQUE，因此任何网站在全球的排名都必须不同。

```
CREATE TABLE website (    id      INT              NOT NULL   AUTO_INCREMENT,    name    VARCHAR(20)      NOT NULL,    url     VARCHAR(30),    age     TINYINT UNSIGNED NOT NULL,    alexa   INT UNSIGNED     NOT NULL   UNIQUE,    uv      FLOAT                       DEFAULT '0',    country CHAR(3)          NOT NULL,    PRIMARY KEY (`id`));
```

如果已经创建了 website 表，则可以通过 ALTER TABLE 语句将 UNIQUE 约束添加到 alexa 字段，代码如下：

```
ALTER TABLE websiteMODIFY alexa INT UNSIGNED NOT NULL UNIQUE;
```

如果您希望为多个字段添加 UNIQUE 约束，则可使用下面的语法：

```
ALTER TABLE websiteADD CONSTRAINT myUniqueConstraint UNIQUE(alexa, url);
```

**删除 UNIQUE 约束**

可以使用下面的 SQL 语句来删除 UNIQUE 约束：

```
ALTER TABLE websiteDROP CONSTRAINT myUniqueConstraint;
```

如果您使用的是 MySQL，那么应该修改为如下的语句：

```
ALTER TABLE websiteDROP INDEX myUniqueConstraint;
```

## 参考致谢

以上内容均摘抄自：[C语言中文网-SQL教程](https://c.biancheng.net/sql/)
