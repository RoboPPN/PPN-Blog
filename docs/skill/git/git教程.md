装机完毕后会用到git这个工具管理代码，下面我来介绍git的一些用法

## 如何创建远程仓库

1、创建SSH Key

在用户主目录下，看看有没有.ssh目录，如果有，再看看这个目录下有没有id_rsa和id_rsa.pub这两个文件，如果已经有了，可直接跳到下一步。如果没有，打开Shell，创建SSH Key：

```bash
ssh-keygen -t rsa -C "youremail@example.com"
```

你需要把邮件地址换成你自己的邮件地址，然后一路回车，使用默认值即可，由于这个Key也不是用于军事目的，所以也无需设置密码。

如果一切顺利的话，可以在用户主目录里找到.ssh目录，里面有id_rsa和id_rsa.pub两个文件，这两个就是SSH Key的秘钥对，id_rsa是私钥，不能泄露出去，id_rsa.pub是公钥，可以放心地告诉任何人。

2、登陆GitHub，创建密钥

打开“Account settings”，“SSH Keys”页面：

然后，点“Add SSH Key”，填上任意Title，在Key文本框里粘贴id_rsa.pub文件的内容。

点“Add Key”，你就应该看到已经添加的Key。

为什么GitHub需要SSH Key呢？因为GitHub需要识别出你推送的提交确实是你推送的，而不是别人冒充的，而Git支持SSH协议，所以，GitHub只要知道了你的公钥，就可以确认只有你自己才能推送。

原文链接：<https://www.liaoxuefeng.com/wiki/896043488029600/896954117292416>

## git如何下载分支代码？

直接用git clone是不行的，它会下载默认分支的代码，如要下载分支代码，则需运行这条命令：

任务一：下载地址为<https://github.com/hemiahwu/vue-basic-playlist.git；分支名为lesson-2>

```bash
git clone -b 分支名 网址.git 

git clone -b lesson-2 https://github.com/hemiahwu/vue-basic-playlist.git
```

原文链接：<https://blog.csdn.net/weixin_43324273/article/details/105687801>

## git配置克隆下来的代码仓库

在将代码克隆下来打算在VSCode上提交修改，出现的错误如下：

```bash
请确保已在Git中配置您的"user.name"和"user.email"
```

```bash
git config user.name <你的github用户名>

git config user.email <你的邮箱>
```

## 参考致谢

- [Git教程-廖雪峰的官方网站](https://www.liaoxuefeng.com/wiki/896043488029600)
