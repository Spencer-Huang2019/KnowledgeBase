# boost::optional 
用于封装可能不存在的值的类型，处理可能为空的对象或值时非常方便。
头文件 #include <boost/optional.hpp>

```cpp
// 初始化
boost::optional<int> opt_int = 10; // 初始化为10
boost::optional<std::string> opt_str = "Hello"; // 初始化为字符串"Hello"
boost::optional<int> opt_int; // 未定义，此时不包含任何值

// 检查值是否存在
if (opt_int) {
    // 如果 opt_int 有值，执行操作
    std::cout << *opt_int << std::endl; // 解引用opt_int获取值
} else {
    // opt_int 没有值
    std::cout << "No value" << std::endl;
}

// 访问
if (opt_int) {
    std::cout << *opt_int << std::endl; // 安全地访问值
}

// 赋值和修改
opt_int = 20; // 赋新值20
opt_int.reset(); // 清空opt_int，使其不包含任何值
// boost::none 表示“无值”状态
opt_int = boost::none; // opt_int现在不包含任何值
// 提供默认值
int value = opt_int.value_or(5); // 如果opt_int有值，返回其值；否则返回5
```
# boost::math::tools::brent_find_minima
用于在给定区间内查找单变量函数的最小值，只适用于单峰情况
```cpp
template <class F>
std::pair<double, double> brent_find_minima(
    F f,
    double lower,
    double upper,
    int digits
);
```

**F f**:  
- 一个可调用对象（函数、函数对象或 lambda 表达式），表示需要最小化的目标函数。
- 必须是单变量函数，即 f(x)。

**int digits**:  
- 搜索精度（以有效位数表示）。例如，digits = 10 表示结果精确到 10 位有效数字。

**注意事项**  
目标函数要求：
- 函数必须是连续的。
- 在给定区间 [lower, upper] 内，函数必须是可优化的（通常需要函数在区间内具有一个唯一的局部最小值）。
搜索区间：
- 如果区间太大，可能会导致算法收敛速度变慢或失败。
- 确保搜索区间包含期望的最小值。
精度参数：
- digits 参数决定了算法的收敛精度。较大的值会增加计算开销，但提高结果精度。
无导数优化：
- 该方法适用于无导数优化问题。如果可以计算导数，建议使用更高效的方法（如梯度下降）。


