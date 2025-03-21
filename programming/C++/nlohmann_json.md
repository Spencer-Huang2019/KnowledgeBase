这个库最常用的是读取 JSON 配置文件了，写入配置文件的情况倒是比较少。  
上次做路端多雷达标定的时候正好遇到了要往 JSON 文件写入的情况。如果只是单层级的写入倒是简单，但是多层级写入或更新就犯难了。  

# 读 JSON 文件
```cpp
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

void read_json(std::string& file_name)
{
    std::ifstream file(file_name);
    if (!file.is_open())
        return;

    json json_data;
    file >> json_data;
    file.close();

    if (json_data.contains("key1"))
    {
        float value = json_data["key1"];
    }

    if (json_data.contains("example_list"))
    {
        for (auto& item: json_data["example_list"])
        {
            if (item.contains("key2"))
                std::cout << item["key2"] << std::endl;
        }
    }
}


```

# 写 JSON 文件
```cpp
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

using json = nlohmann::json;

struct MyStruct
{
    int id;
    std::string name;

    // 将 MyStruct 转换为 JSON 对象
    operator json() const
    {
        return {{"id": id}, {"name": name}};
    }
};

void write_json(std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open())
        return;
        
    json json_data;
    file >> json_data;
    file.close();

    std::vector<MyStruct> my_structs = {
        {1, "Alice"},
        {2, "Bob"}
    };

    json array = json::array();
    for (const auto& ms: my_structs)
    {
        array.push_back(ms);    // push back 的时候，编译器会自动调用 operator nlohmann::json() 类型转换运算符，将  转换为 nlohmann::json 类型的对象
    }

    json_data["myArray"] = array;
    std::ofstream(file_path) << json_data.dump(4);
}

```
``` json
{
    "myArray": [
        {
            "id": 1,
            "name": "Alice"
        },
        {
            "id": 2,
            "name": "Bob"
        }
    ]
}
```