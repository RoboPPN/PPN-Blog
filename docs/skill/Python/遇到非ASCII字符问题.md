
问题描述：

```bash
SyntaxError: Non-ASCII character '\xe5' in file /home/ppn/agilex_ws/src/joint_state_pub/script/test.py on line 6, but no encoding declared; see http://python.org/dev/peps/pep-0263/ for details
```

问题分析：

这个错误是Python解释器遇到了非ASCII字符的问题，并且没有声明文件的编码方式。根据错误信息，它指出了具体的文件路径和行号。

在Python中，默认情况下，源代码文件应该使用ASCII编码，如果你在文件中使用了非ASCII字符（如中文、日文、俄文等），你需要在文件的第一行或第二行添加一个特殊的注释来声明文件的编码方式。

在你的情况下，你可以尝试在你的Python脚本文件的开头添加以下注释来声明编码方式为UTF-8：

```python
# -*- coding: utf-8 -*-
```

这行注释的作用是告诉Python解释器，使用UTF-8编码来解析源代码文件。你可以根据你的实际编码方式进行修改。

请注意，这些注释必须位于文件的开头，并且在任何其他代码之前。添加了这些注释之后，再次运行你的脚本应该就不会再出现这个错误了。

同时，确保你的Python解释器的版本也是支持Unicode字符的，最好使用较新的Python版本。
