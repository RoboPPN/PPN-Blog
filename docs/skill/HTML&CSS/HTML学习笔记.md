### 什么是HTML？
- HyperText Markup Language（超文本标记语言）
- 不是编程语言
- 告诉浏览器如何构造网页

[HTML学习链接](https://developer.mozilla.org/en-US/docs/Web/HTML)

### 开始
![img error](https://i.bmp.ovh/imgs/2021/10/953a1d2ceb2eb032.png)

上面是HTML一行简单的代码，它包括了`开始标签（Opening Tag）`和`结束标签（Closing Tag）`，以及中间用尖括号包括起来的`文本内容（Content）`，`<p>`就是一个Tag，HTML里面有很多种Tag

下图列举了几个常用的Tag

![img error](https://i.bmp.ovh/imgs/2021/10/06c0354ce9fa21b9.png)

 [点击了解更多Tag](https://developer.mozilla.org/en-US/docs/Web/HTML/Element)

 ### HTML的文件组成
 ![img error](https://i.bmp.ovh/imgs/2021/10/4880ea636bf4088b.png)

 `<head>`部分通常放一些不被渲染在网页主体的内容，比如可以在`<head>`上放一些网页的基本信息

 可用于`<head>`元素内的元素有: `<title>, <base>, <link>, <style>, <meta>, <script>, <noscript>, <command>`

 `<body>`则是放一些渲染给用户看的内容，比如可以放`<h1>(标题)`、`<p>(段落)`

 这里说一下`<h>`有`<h1~h6>`,代表了各个级别的标题

 `<p>`就是paragraph（段落）的意思，代表了一个段落



 ### 块级元素＆内联元素
 ![img error](https://i.bmp.ovh/imgs/2021/10/c045bdafa3076f37.png)

 [点击查看特定标签属性](https://developer.mozilla.org/en-US/docs/Web/HTML/Attributes)

 [点击查看全局标签属性](https://developer.mozilla.org/en-US/docs/Web/HTML/Global_attributes)


### Forms表单
```HTML
<form action="form.js" method="POST">
    <div>
        <label>First name</label>
        <input type="text" name="姓名" placeholder="输入你的姓名">
    </div> 
    <input type="submit" name="submit" value="提交">   <!--创建一个提交按钮-->
</form>
```

上面代码在网页创建一个输入名字的表单，`action="form.js"`的意思是内容会提交到`form.js`这里



附上[学习链接](https://www.bilibili.com/video/BV1vs411M7aT/?spm_id_from=333.788.videocard.0)

