很多东西都是在实践中进行学习，遇到问题就记录下来了，即使有些问题确实很基础。。。但长时间积累下来，总能慢慢变精的。

# 编译相关
**基类构造函数有两个参数，派生类构造函数也是这两个参数，派生类的构造函数要自己实现，那么头文件和源文件的构造函数怎么写，出现了 redefinition 的问题**
派生类的头文件里写了构造函数 Derived(float a, int b): Base(a, b){}, 然后 cpp 里又写了一遍，自然是重复定义了。正确的写法是，要么在 头文件直接实现，要么头文件先声明，后 cpp 实现，声明是这样写的 Derived(float a, int b);

**/usr/bin/ld: warning: libboost_filesystem.so.1.71.0, needed by /opt/ros/noetic/lib/librosbag_storage.so, may conflict with libboost_filesystem.so.1.65.0**
ROS 依赖的是 1.17 版本，这个 apt 已经安装过了，在 /usr/lib/x86_64-linux-gnu 下。然后源码安装的 1.65 版本在 /usr/local/lib 下边，这个是系统 PCL 的依赖。所以没法直接删除 1.65 版本。LD_LIBRARY_PATH 添加了 /usr/lib/x86_64-linux-gnu 也没法解决问题。这个问题比较复杂了，后边的各种手段还遇到了内存分配相关的问题。
主要原因还是在于，工程既依赖 ROS，又依赖 PCL，然后这 PCL 版本还是源码安装的 PCL 1.8，同时之前在做工程迁移的时候，也没考虑到为什么非得用 PCL 1.8，然后就导致系统有两个 PCL 版本，通常来说，也不会有影响的，只要依赖结构够清晰，CMakeLists 写对了就没事。可问题是，非用 PCL 1.8 的缘由是不存在的，整体的工程代码包的依赖关系也没有文档之类的做详细的说明，到后来出现内存分配问题也很难调查到底是什么原因，只知道删掉 PCL 1.8 就好了。程序并不是非得使用 PCL 1.8，使用 ROS 自带版本的 PCL 就足够了。环境问题，确实比较不好深究具体原因，只能知道个大概，然后找个方案解决了。碰到这个问题的教训就是，搭环境的时候不好把环境搞得太复杂，边搭边记录，遇到版本冲突时，应该先考虑为什么要使用不一样的版本，然后再考虑如何解决。如果非用特定的版本，最后在 CMakeLists 里 find_package 写清楚版本号。

**vector Map, set 元素使用自定 结构体或类时。工程代码写得不是很标准，若不是遇到上边所说的内存分配问题，我还真不知道还有这样的防止内存分配问题的方案**
基本信息：有个自定义的结构体，内部包含了 Eigen::Matrix, cv::Vec，以及其他一些基本类型。结构体内部没有任何处理内存对齐的措施，Vector，Map 和 Set 包含这个结构体时也是直接使用的。之前的程序跑着都正常，倒也没出现过内存分配异常的问题。后来是因为程序里得依赖 ROS 相关的包，这才出现内存分配异常的问题。一开始大模型给的解决方案就是结构体内存对齐处理。  
```cpp
    // 原来的写法
    struct Example
    {
        int a;
        float b;
        bool c;
        int d;

        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        cv::Vec<float, 7> box;

        double e;
    };

    // 上边的写法有几点需进行改进
    // 1. 对于 Eigen：：Matrix 和 cv::Vec 这种有最大字节对齐要求的类型，应该放在前边。
    // 2. 相同数据类型的排在一起
    // 3. 对应 Eigen 数据类型，应该进行特殊处理
    struct ExampleModified
    {
        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        cv::Vec<float, 7> box;

        double e;
        int a;
        int d;
        float b;
        bool c;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template <typename T>
    using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

    int main()
    {
        std::vector<ExampleModified, Eigen::aligned_allocator<ExampleModified>> my_vec;
        // 或
        AlignedVector<ExampleModified> my_vec;

        return 1;
    }
```

**编译信息输出到日志，有时候，终端的信息刷得太快，找不到报错的地方了**
make -j16 2>&1 | tee build.log  
在 Linux 系统中，每个进程有三个标准的文件描述符：标准输入（0），标准输出（1），标准错误（2）。2>&1 是一个重定向操作，将标准错误输出重定向到标准输出，那么 make 过程中产生的 error 信息和正常的信息都会合并到标准输出流中。 | 用于将一个命令的输出作为另一个命令的输入。tee 会将收到的数据，一部分显示在终端屏幕，另一部分写入指定的文件。

**Boost system 链接时报错“boost::system::generic_category()未定义的引用”**
解决方法，在 cmakelists.txt 文件里添加
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
在引入 rtk 的代码之后，反而因为这句话编译时报下边的错了。。。（解决方法是把上边 add_definitions 注释掉就好了）
multiple definition of `boost::system::throws'  
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY) 表示boost.System 库会以仅头文件的方式使用，不需要链接 boost.System 库的动、静态库文件，所有的实现都包含在头文件中。  
RTK 的代码应是使用了 Boost.System 的链接库，所以出现了重复定义的问题。  
Boost system 链接时报错“boost::system::generic_category()未定义的引用 这个问题的出现实际上是因为没有正确地指定链接库导致的，所以加上 add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY) 能解决。引入 RTK 代码之后这个问题被解决了正式因为 RTK 模块正确指定了链接库。

**编译时出现 multiple definition of “...”，两个 cpp 都调用了同一个头文件**
使用 ifndef 之后还是出现了同样的问题，头文件只声明但不定义，还是不行。。最后是因为两个 cpp 里存在包含关系，把外层的头文件引用去掉就可以了

**如果没有 using namespace std; 在调用 abs 方法的时候，如果没有用 std::abs() 那么可能会出现非绝对值计算的结果**

**C++ 类模版，如果用 .h 和 .cpp 文件的形式，那么会出现链接报错**  
声明和定义需要放在同一个文件里，因为编译器在编译的时候需要明确知道模版的数据类型是什么；改成使用 .h 和 .hpp 的方式就不会出现链接报错了。另一个是，如果 A 是模版类，B继承了 A 类要传入模版参数，虽然 B 类很像模版类，但是可以用 .h 和 .cpp 方式。

**A 类继承 B 类，B 类有两个纯虚函数，C 类又继承 A 类，其中 A 类型实现了其中一个纯虚函数， C 类实现了另一个 纯虚函数，编译时报错 error: expected class-name before '{' token**  
主要原因是，A 类是一个模版类，C 类继承 A  类的时候是按普通的方法继承的，因此报错了。C 类继承 A 类时应该按如下方式
```cpp
template <typename T>
class C : public A<T>
{
    // 构造函数
    C() : A<T>() {}
}
```

**没有继承模版类的情况下，又一次出现 编译时报错 error: expected class-name before '{' token**  
循环依赖导致的，即头文件 A 引入了头文件 B，而头文件 B 又引入了头文件 A，导致编译器在处理时遇到无法解决的依赖关系。
解决方法：
头文件 B 引入 头文件 A 时，可以直接放在 cpp 里

**C++ 编译报错  in function adapter::translate::toRadians(float)': traffic_incident.cpp:(.text+0xc0): multiple definition of adapter::translate::toRadians(float)'; CMakeFiles/roadside.dir/src/incident_pipeline.cpp.o:incident_pipeline.cpp:(.text+0x480): first defined here**
产生原因：如果函数的定义放在头文件中，并且头文件在多个源文件中被包含，就会导致函数定义在每个源文件中都有，造成重复定义。
解决方案：
1. 定义和声明分别放在 cpp 和 h 文件中，不要都写在头文件里
2. 使用 inline 或 static 关键字来避免链接时出现多个定义错误

**terminate called after throwing an instance of 'std::logic_error'   what():  basic_string::_M_construct null not valid**  
通常表明程序在尝试使用一个空指针（nullptr）来构造 std::string 对象时，遇到了问题。

**undefined reference to `vtable for adapter::roadside::Incident'。StopIncident 继承 Incident，Incident 继承 stage，stage 有两个纯虚函数，Incident 里边实现了，Incident 里也有两个自己的纯虚函数，StopIncident 里边实现了。**
通常原因：有纯虚函数没有被派生类实现
Incident 重写虚函数放在 h 文件里是没问题的，但是一旦放到 cpp  文件里，编译就会报这个问题。
把 cpp 文件改成 cc 文件，内容不修改，不会报这个错。。。
重新把 cc 改回 .cpp 又可以了，可能是之前文件名写错了，cmake list 没有识别到 cpp 文件。。。
重新执行一下 cmake ..  
其实吧，是因为虽然 cmakelist里查找源文件时使用的 GLOB，当有新的源文件添加的时候，应该重新 cmake，直接执行 make 命令编译很多时候是不会包含新增的源文件的。  

**typename boost::detail::sp_member_access<T>::type = adapter::roadside::Life*]: Assertion `px != 0' failed.**
~~原因：智能指针没有正确初始化~~
~~智能指针不能直接初始化为 nullptr，也不能直接就 boost::shared_ptr<Type> ptr;~~
智能指针可以初始化为 nullptr 的。主要问题还是出在当跟踪框数量只有 0 的时候，没有退出函数，而是继续往下执行，往下执行的时候会出现空指针被使用的情况。

**A和B继承Base，如果一个容器如 vector 要存储A 或 B 的实例，那么需要用指针来管理。**  
```cpp
std::vector<std::shared_ptr<Base>> tmp_ptrs;
tmp_ptrs.push_back(std::make_shared<A>(...));
```

**C++ 工程中，有些会把声明放在一个 头文件，然后会把定义放在另一个头文件，为什么这么做？**
1. .h 和 .hpp 的区别
  a. 都是头文件的扩展名，主要区别在于使用习惯和代码组织方式
  b. .hpp 强调“只适用于C++”，常用于模版、类、命名空间等
2. .cc 和 .cpp 的区别
  a. 都是源文件扩展名，适用于 Linux、GCC/G++ 编译环境
  b. .cc 早期 UNIX/Linux 代码风格，.cpp 是现代 C++ 标准扩展名，适用于 跨平台代码、windows/MSVC
  
模版类的定义和实现必须在同一个文件中，一般都在头文件中实现，比如 .hpp 文件。模板代码如果只提供 .h 而不提供源码，编译器无法在其他项目中实例化模板类型，导致链接错误。
**如果程序需要 install 这个头文件，那么源码就会暴露。有什么方法解决呢？**
1. 使用 PImpl 模式（使用了嵌套类）
```cpp
// my_template.h (只暴露接口)
#ifndef MY_TEMPLATE_H
#define MY_TEMPLATE_H

#include <memory>

template<typename T>
class MyTemplate {
public:
    MyTemplate(T value);
    void show();
    ~MyTemplate();
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;  // 指向实现的指针
};

#endif // MY_TEMPLATE_H
```
```cpp
// my_template.cpp (隐藏实现)
#include "my_template.h"
#include <iostream>

template<typename T>
class MyTemplate<T>::Impl {
public:
    Impl(T value) : data(value) {}
    void show() { std::cout << data << std::endl; }
private:
    T data;
};

template<typename T>
MyTemplate<T>::MyTemplate(T value) : pImpl(std::make_unique<Impl>(value)) {}

template<typename T>
void MyTemplate<T>::show() { pImpl->show(); }

template<typename T>
MyTemplate<T>::~MyTemplate() = default;

// 只实例化 int 和 double
template class MyTemplate<int>;
template class MyTemplate<double>;
```

2. 如果将声明写在了 .h 文件，定义写在了 .hpp 文件，即使只 install .h 文件，用户在使用的时候光有 .h 文件是不够的，还得 include .hpp 文件，这么做相当于还是暴露了源码。
3. 模版类显示实例化分离
  a. 隐藏模版实现细节，减少模版编译时间（避免在多个编译单元重复实例化），提供预编译好的模版实例
  b. 其实上边的 2 中，再加上一个 .cpp 文件显示实例化模版就可以了

install 头文件只要 install A.h 即可，B.h 不用 install，这样不会暴露源码，其次 cpp 进行了显示实例化，链接不会发生报错。这种模式是在 FCL 大型开源库里见到的

```cpp
// A.h
#ifndef A_H
#define A_H

#include "B.h"  // 让 B.h 提供模板的定义

template <typename T>
class DynamicAABBTreeCollisionManager {
public:
    DynamicAABBTreeCollisionManager();
    void someMethod();
};

#endif  // A_H
```
```cpp
// B.h
#ifndef B_H
#define B_H

#include "A.h"  // 让 A.h 提供模板的声明
#include <iostream>

template <typename T>
DynamicAABBTreeCollisionManager<T>::DynamicAABBTreeCollisionManager() {
    std::cout << "Constructor called" << std::endl;
}

template <typename T>
void DynamicAABBTreeCollisionManager<T>::someMethod() {
    std::cout << "Method called" << std::endl;
}

#endif  // B_H
```
```cpp
// C.cpp
#include "B.h"

// 显式实例化 double 类型
template class DynamicAABBTreeCollisionManager<double>;
```