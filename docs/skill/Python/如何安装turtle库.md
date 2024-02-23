当我使用`pip3 install turtle`时，终端会出现如下错误：

```bash
Collecting turtle
  Using cached turtle-0.0.2.tar.gz (11 kB)
    ERROR: Command errored out with exit status 1:
     command: /usr/bin/python3 -c 'import sys, setuptools, tokenize; sys.argv[0] = '"'"'/tmp/pip-install-fn6bnbs9/turtle/setup.py'"'"'; __file__='"'"'/tmp/pip-install-fn6bnbs9/turtle/setup.py'"'"';f=getattr(tokenize, '"'"'open'"'"', open)(__file__);code=f.read().replace('"'"'\r\n'"'"', '"'"'\n'"'"');f.close();exec(compile(code, __file__, '"'"'exec'"'"'))' egg_info --egg-base /tmp/pip-install-fn6bnbs9/turtle/pip-egg-info
         cwd: /tmp/pip-install-fn6bnbs9/turtle/
    Complete output (6 lines):
    Traceback (most recent call last):
      File "<string>", line 1, in <module>
      File "/tmp/pip-install-fn6bnbs9/turtle/setup.py", line 40
        except ValueError, ve:
                         ^
    SyntaxError: invalid syntax
    ----------------------------------------
ERROR: Command errored out with exit status 1: python setup.py egg_info Check the logs for full command output.
```

根据终端提示信息，我们知道pip3在下载turtle 0.0.2包后，会解压到本地再安装，提示的错误在解压的setup.py文件里面有语法错误`except ValueError, ve:`，这句是python2的写法，没有括号，放到python3环境就不适用了，将这句改成`except (ValueError, ve):`即可

解决方案：按照给定的链接---->[turtle 0.0.2库](https://pypi.org/project/turtle/#files)，把turtle包下载到本地，手动解压，修改setup.py文件再安装。

1. 打开setup.py文件，将第40行修改为`except (ValueError, ve):`
2. 用`pip3`安装：

   ```bash
    pip install -e turtle-0.0.2
    ```

    `-e`后面接上我们修改过setup.py文件的目录。
3. done!
4. 如果提示`python-tk`未安装，用`apt`命令安装就可以了：

   ```bash
   sudo apt install python-tk
   ```
