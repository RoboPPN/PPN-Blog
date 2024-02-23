## 使用python加载json数据

```python
import json

json_data = '''
{
  "serialIO": [
    {
      "id": 1,
      "position": "Pin1",
      "name": "lamp",
      "value": false
    },
    ...
  ]
}

data = json.loads(json_data)
for item in data["serialIO"]:
            if item["position"] == "Pin1" and item["value"] == True:
                print("xxxxxxxxxx")
                self.open_pin("Pin1")
```

## 读取yaml文件内容并将其转换成json格式

使用Python的yaml库来读取yaml文件，并将其转换为对应的JSON格式。以下是一个示例代码：

```
import yaml
import json

# 读取yaml文件
with open('relay.yaml', 'r') as file:
    yaml_data = yaml.safe_load(file)

# 转换为JSON格式
json_data = json.dumps(yaml_data)

# 输出JSON数据
print(json_data)
```

这段代码会将名为`relay.yaml`的文件内容读取并解析为Python对象。然后，使用`json.dumps()`方法将Python对象转换为JSON格式的字符串。

请确保在运行代码之前安装了`yaml`库，可以通过运行`pip install pyyaml`来安装。

## 将json转换成yaml格式并写入relay.yaml文件

```py
import yaml
import json

def convert_to_yaml(json_data):
    # 解析JSON数据
    data = json.loads(json_data)

    # 转换为YAML格式
    yaml_data = yaml.dump(data, default_flow_style=False)

    # 将YAML数据写入文件
    with open('relay.yaml', 'w') as file:
        file.write(yaml_data)
```

## # 将json数据写入文件，使用indent=4参数保持整齐格式

```python
with open('./config/relay.json', 'w') as file:
    json.dump(data, file, indent=4)
```

## python  os.path.basename()函数

代码如下：

```py
device_name = os.path.basename(device_path)
```

​ 具体来说，`os.path.basename()` 是一个函数，它接受一个路径字符串作为参数，并返回该路径的基本名称部分。基本名称是指路径中最后一个目录或文件的名称，不包括任何目录路径。

​ 例如，假设 `device_path` 的值是 `/dev/ttyUSB0`，那么 `os.path.basename(device_path)` 将返回 `'ttyUSB0'`，并将其赋值给 `device_name` 变量。

## 我最近在使用pip install 安装某个包，我本想安装在.local/lib/python2.7目录下的，但每次它都给我装到.local/lib/python3.6目录下

问题可能出在您的Python环境配置上。在Python 3.3+版本中，`pip`默认会安装包到Python 3.x的`.local/lib/python3.x/site-packages`目录下，而不是Python 2.7的目录。

为了确保包被正确安装到Python 2.7的`.local/lib/python2.7`目录下，您可以使用Python 2.7的`pip`来进行安装。请确保在安装时使用了正确的`pip`版本。

1. 首先，确定您正在使用Python 2.7版本，可以通过运行以下命令检查Python版本：

   ```
   
   python --version
   ```

   如果输出类似于 `Python 2.7.x`，则表示您当前使用的是Python 2.7。

2. 接下来，确保您正在使用Python 2.7的`pip`。在命令行中运行以下命令：

   ```
   
   python -m pip --version
   ```

   如果输出中包含类似于 `python 2.7` 的字样，表示您的pip版本是与Python 2.7关联的。

3. 最后，使用Python 2.7的`pip`来安装您的包。在命令行中运行：

   ```
   
   python -m pip install your-package-name
   ```

   这将使用Python 2.7的pip安装包，并将其安装到正确的目录下。

请注意，如果您的系统同时安装了多个Python版本，并且配置混乱，可能导致`pip`的行为不一致。在这种情况下，您可能需要检查和修复您的环境配置，以确保正确使用Python 2.7的pip。
