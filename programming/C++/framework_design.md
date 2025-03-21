主要记录一些常用的，以及其他源代码里写得好的一些设计

# 设计模式
## 创建型模式
处理对象创建问题
### 单例模式
保证几个类仅有一个实例，并提供一个全局访问点  
使用场景主要有：数据库连接池、日志管理、线程池   

```cpp
class Singleton{
public:
    static Singleton* get_instance()
    {
        static Singleton* instance = nullptr;
        return instance;
    }

private:
    Singleton(){}
    Singleton(const Singleton&) = delete; // 显示的禁用该构造函数，删除拷贝构造函数
    Singleton& operator=(const Singleton&) = delete; // 删除拷贝赋值运算符
};
```
上边的写法可以做一个改进，保证在多线程的情况下，只会被调用一次。如下边的宏里的 std::call_once。std::nothrow 的作用是如果内存分配失败，不抛出异常，而是返回一个 nullptr  

**用宏来声明一个单例模式**
```cpp
#define NOT_ALLOWED_COPY_AND_ASSIGN(classname) \
    classname (const classname&) = delete; \
    classname& operator=(const classname&) = delete; 

#define DECLARE_SINGLETON(classname) \
    public: \
        static classname* get_instance() {\
            static classname* instance = nullptr; \
            if (instance == nullptr) { \
                static std::once_flag flag; \
                std::call_once(flag, [&]{ instance = new (std::nothrow) classname(); }); \
            } \
            return instance; \
        } \

    private: \
        classname(); \
        NOT_ALLOWED_COPY_AND_ASSIGN(classname) \
```

```cpp
class Singleton
{
public:
    Singleton(){}
    void foo();

private:
    DECLARE_SINGLETON(Singleton)
};
```

### 工厂方法模式
定义一个接口，让子类决定实例化哪一个具体类
```cpp
class Product {
public:
    virtual void use() = 0;
    virtual ~Product() = default;
};

class ConcreteProductA : public Product {
public:
    void use() override { std::cout << "Using Product A" << std::endl; }
};

class Factory {
public:
    static std::unique_ptr<Product> create_product() {
        return std::make_unique<ConcreteProductA>();
    }
};
```

### 抽象工厂模式
提供一个接口，创建一系列相关的对象，而不指定具体类
创建一个基类数据类型的工厂，派生类在自己的声明文件中进行注册，使用时根据实际调用的类名来确定使用哪个类的实例。
```cpp
// base.h
class Base
{
public:
    void foo();
};
```
```cpp
// factory.h
class Factory
{
public:
    using Creator = std::function<std::unique_ptr<Base>()>;

    static Factory* get_instance()
    {
        static Factory* instance = nullptr;
        return instance;
    }

    void register_class(const std::string& name, Creator creator)
    {
        register_[name] = std::move(creator);
    }

    std::unique_ptr<Base> create_class(const std::string& name)
    {
        auto it = register_.find(name);
        if (it != register_.end())
        {
            return it->second;
        }
        else
        {
            return nullptr;
        }
    }

private:
    std::unordered_map<std::string, Creator> registry_;

    Factory() = default;
    ~Factory() = default;
    Factory(const Factory&) = delete;
    Factory& operator=(const Factory&) = delete;
};
```

```cpp
// register_macro.h
#include "factory.h"

#define REGISTRE_CLASS(classname) \
    static bool classname##Register = []() { \
        Factory::get_instance().register_class(#classname, [](){ \
                return std::make_unique<classname>(); \
            }) \
        return true; \
    }();

```
```cpp
// derived.h
class Derived: public Base
{
public:
    Derived():Base(){}
    void foo();

};

REGISTER_CLASS(Derived)
```

## 结构型模式
### 适配器模式
将不兼容的接口转换成可用接口  
适用场景：老代码复用，第三方库兼容  

### 装饰器模式
动态添加功能，而不改变原始类  
适用场景：I/O 流增强、日志功能增强  

# 用宏定义指针别名
```cpp
#include <memory>

#define DECLARE_PTR(name, type) \
    typedef std::shared_ptr<type> name##Ptr; \
    typedef std::shared_ptr<const type> name##ConstPtr; \
    typedef std::unique_ptr<type> name##UniquePtr; \
    typedef std::unique_ptr<const type> name##ConstUniquePtr; \
    typedef std::weak_ptr<type> name##WeakPtr; \
    typedef std::weak_ptr<consttype> name##WeakConstPtr; 

#define CLASS_FORWORD(C) \
    class C; \
    DECLARE_PTR(C, C)

#define STRUCT_FORWORD(C) \
    struct C; \
    DECLARE_PTR(C, C)
```
```cpp
CLASS_FORWARD(Example);
class Example
{

};
```

# Export 宏
这个是在 FCL 开源库里看到的，感觉也挺妙的。  
Cmakelists 里
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME export.h)
用于自动生成一个头文件（如 export.h），该文件包含了用于导出和导入共享库符号（例如类、函数等）的宏。
这个宏通常是 __declspec(dllexport)（在 Windows 上）或类似的标识符，这取决于编译环境。（设${PROJECT_NAME} = MYLIBRARY_API）
// 相当于
```cpp
#ifdef MyLibrary_EXPORTS
#define MYLIBRARY_API __declspec(dllexport)
#else
#define MYLIBRARY_API __declspec(dllimport)
#endif
```
使用
```cpp
// MyClass.h
#include "export.h"

class MYLIBRARY_API MyClass {
public:
    MyClass();
    void myMethod();
};


template <typename S>
FCL_EXPORT
S distance(...);
```

