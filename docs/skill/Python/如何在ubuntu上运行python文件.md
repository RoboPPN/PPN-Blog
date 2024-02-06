
在Ubuntu无需编译.py文件即可运行它。

Python是一种解释型语言，您可以直接运行脚本，使用：
```shell
python hello.py
```
或者通过将`#!/usr/bin/env python`添加到脚本的顶部使脚本可执行，给`.py`文件添加权限然后运行：
```shell
./hello.py
```

:::tip
关于脚本第一行的 `#!/usr/bin/python` 的解释，相信很多不熟悉 `Linux` 系统的同学需要普及这个知识，脚本语言的第一行，只对 `Linux/Unix` 用户适用，用来指定本脚本用什么解释器来执行。

有这句的，加上执行权限后，可以直接用 `./` 执行，不然会出错，因为找不到 `python` 解释器。

`#!/usr/bin/python` 是告诉操作系统执行这个脚本的时候，调用 `/usr/bin` 下的 `python` 解释器。

`#!/usr/bin/env python` 这种用法是为了防止操作系统用户没有将 `python` 装在默认的 `/usr/bin` 路径里。当系统看到这一行的时候，首先会到 `env` 设置里查找 `python` 的安装路径，再调用对应路径下的解释器程序完成操作。

`#!/usr/bin/python` 相当于写死了 `python` 路径。

`#!/usr/bin/env python` 会去环境设置寻找 `python` 目录，可以增强代码的可移植性，`推荐这种写法`。

分成两种情况：

- 如果调用 python 脚本时，使用:
```shell
python script.py 
```
`#!/usr/bin/python` 被忽略，等同于注释

- 如果调用python脚本时，使用:
```shell
./script.py 
```
`#!/usr/bin/python` 指定解释器的路径

:::
