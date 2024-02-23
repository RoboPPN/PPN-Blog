# MySQL workbench中PK、NN、UQ、BIN、UN、ZF、AI、G这些选项代表的意思是？

- PK：primary key (column is part of a pk) 主键，一张表里只能有一个，且设置主键的字段必须同时勾上NN，不然会报错
- NN: not null (column is nullable) 是否为空 （非空）
- UQ: unique (column is part of a unique key) 唯一索引 （外键用来建立与其他表的联系）
- AI: auto increment (the column is auto incremented when rows are inserted) 自动增加
- BIN: binary (if dt is a blob or similar, this indicates that is binary data, rather than text) 二进制数据（比text更大）
- UN: unsigned (for integer types, see docs: “10.2. Numeric Types”) 无符号整型（非负数）
- ZF: zero fill (rather a display related flag, see docs: “10.2. Numeric Types”) 补零 填充0 例如字段内容是1 int(4), 则内容显示为0001
- G: Generated Column （mysql5.7新特新）：这一列由其他列计算而得
