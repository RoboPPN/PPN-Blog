### Block、Inline、Inline-Block
HTML大致上分为Block、Inline以及Inline-Block

所有HTML属性都可通过display设定为block、inline或inline-block

[了解Block、Inline、Inline-Block](https://www.bilibili.com/video/BV1mk4y197se?spm_id_from=333.999.0.0)


### CSS Box model
CSS Box Model总共包括4大属性:Content/Padding/Border/Margin


1. Border：框线
   
    Border指的是框线
    ![img error](https://img-blog.csdnimg.cn/0a5fc8febc6f4b82ac9579fcb2f6a0d6.png)
    由上图可以很直观的看到Border就是一条6px的黑色框线


2. Padding
   
    Padding指的是内容与框线之间的距离
    ![img error](https://img-blog.csdnimg.cn/ee1ec5d4211e4b9ebafcc4cac19f6b42.png)
    上图可知新增了padding:30px之后，内容与框线之间的上下左右，都增加了20px的空间

3. Margin
   
   Margin指的是框线与其他元素之间的距离
   ![img error](https://img-blog.csdnimg.cn/7b1b3da3328c4297819f436147b9a3b8.png)


### Padding&Margin参数说明
当Padding或Margin里面：

1. 只有一个元素10px时，它表示的是同时设定到上下左右的距离
2. 有两个元素10px  20px时，它表示的是设定到上下、左右的距离
3. 有三个元素10px 20px 30px时，它表示的是设定到上、左右、下的距离
4. 有四个元素10px 20px 30px 40px时，它表示的是从上方向开始，顺时针方向，设定上、右、下、左的距离

以及还有Padding&Margin加上-top、-left、-right、-bottom，方便调整某个方向时使用




