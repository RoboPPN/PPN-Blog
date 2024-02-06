### 什么是CSS?
- Cascading Stylesheets(层叠样式表)
- 不是编程语言
- 告诉浏览器如何指定样式、布局
- 样式定义如何显示 HTML 元素
- 样式通常存储在样式表中
- 外部样式表可以极大提高工作效率
- 外部样式表通常存储在 CSS 文件中
- 多个样式定义可层叠为一个

[CSS学习链接](https://developer.mozilla.org/en-US/docs/Learn/CSS)


### 一段CSS简单内容
![img error](https://s3.bmp.ovh/imgs/2021/10/170cb97fa42a2617.png)

上面的例子是使用`p`这个选择器将`<p>`里面的文字变成红色；

CSS中有非常多的选择器;


### 三种添加CSS的方式
![img error](https://s3.bmp.ovh/imgs/2021/10/72cbbb7ceab92f8f.png)


- 外部样式表：当样式需要应用于很多页面时，外部样式表将是理想的选择。在使用外部样式表的情况下，你可以通过改变一个文件来改变整个站点的外观。每个页面使用 `<link>` 标签链接到样式表。 `<link>` 标签在（文档的）头部：
```html
<head>
<link rel="stylesheet" type="text/css" href="mystyle.css">
</head>
```
浏览器会从文件 mystyle.css 中读到样式声明，并根据它来格式文档。

外部样式表可以在任何文本编辑器中进行编辑。文件不能包含任何的 html 标签。样式表应该以 .css 扩展名进行保存。下面是一个样式表文件的例子：
```html
hr {color:sienna;}
p {margin-left:20px;}
body {background-image:url("/images/back40.gif");}
```


- 内部样式表：当单个文档需要特殊的样式时，就应该使用内部样式表。你可以使用 `<style>` 标签在文档头部定义内部样式表，就像这样:

```html
<head>
<style>
hr {color:sienna;}
p {margin-left:20px;}
body {background-image:url("images/back40.gif");}
</style>
</head>
```

- 内联样式：由于要将表现和内容混杂在一起，内联样式会损失掉样式表的许多优势。请慎用这种方法，例如当样式仅需要在一个元素上应用一次时。要使用内联样式，你需要在相关的标签内使用样式（style）属性。Style 属性可以包含任何 CSS 属性。例如：
```html
<p style="color:sienna;margin-left:20px">这是一个段落。</p>
```


[点击查看更多CSS选择器](https://developer.mozilla.org/en-US/docs/Learn/CSS/Introduction_to_CSS/Selectors)


### CSS中6种选择颜色方式
![img error](https://s3.bmp.ovh/imgs/2021/10/2a024dc8d2737c49.png)

[点击查看更多颜色取值](https://developer.mozilla.org/en-US/docs/Web/CSS/color_value)

[颜色转换工具](https://serennu.com/colour/hsltorgb.php)


### 字体
![img error](https://s3.bmp.ovh/imgs/2021/10/af7aa9260b951ea3.png)

[点击查看更多字体](https://developer.mozilla.org/en-US/docs/Learn/CSS/Styling_text/Fundamentals)



### [CSS选择器](https://www.runoob.com/cssref/css-selectors.html)




