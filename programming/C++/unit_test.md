# 前言

虽然有过 5 年左右的软件测试工程师经验，但是那些年里几乎不需要我写代码，主要还是功能测试，接口测试，然后分析问题的时候一般也不用代码，更不用说写 C++ 了。  

虽然后来自己学习了 python pytest + allure 的自动化测试框架，但也不涉及单元测试。所以 python 的单元测试我也没有写过。  

但是现在做了算法工程师，使用最多的是 C++ 语言，实际工作的工程里也没见着单元测试的影子。每次自己新增模块或方法的时候，一旦出问题，gdb 都不好定位，只能是不断地 std::cout ，其实吧，效率确实是低了。特别是有些时候就想验证一下函数的计算逻辑对不对时，还得专门写一个 main，属实是有点 low 了。  

这次，我是真要好好试试 C++ 的单元测试了。  

# Google Test(GTest)

安装： sudo apt install libgtest-dev  

## 配置 CMakeLists

find_package(GTest REQUIRED)  

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(my_test test_sum.cpp sum.cpp)  # 把要测试的函数所在源文件和 写的 gtest cpp 文件都包含  

target_link_libraries(my_test PRIVATE GTest::GTest GTest::Main)

## test 文件

### TEST()宏的基本格式

TEST(TestCaseName, TestName){}  

**TestCaseName** ：测试用例组（可以理解为测试类别）

**TestName**：具体的测试名称（针对某个特定场景）

TEST() 不能写在类的成员函数里，必须是全局函数

```cpp
// test_sum.cpp
#include <gtest/gtest.h>
#include "sum.h"

// 测试 sum() 函数
TEST(SumTest, HandlesPositiveNumbers) {
    EXPECT_EQ(sum(2, 3), 5);   // 2+3=5，测试通过
    EXPECT_EQ(sum(0, 0), 0);
}

TEST(SumTest, HandlesNegativeNumbers) {
    EXPECT_EQ(sum(-2, -3), -5);
    EXPECT_EQ(sum(-2, 2), 0);
}

// main 函数
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

```

### TEST_F() 宏（适用于 Fixture 测试）

如果多个测试用例需要共享相同的测试数据/环境，可以使用 TEST_F()，需要继承 ::testing::Test  

```cpp
#include <gtest/gtest.h>

class MathTest : public ::testing::Test {
protected:
    void SetUp() override {  // 测试前运行
        a = 2;
        b = 3;
    }
    int a, b;
};

// 使用 TEST_F 访问类成员
TEST_F(MathTest, Addition) {
    EXPECT_EQ(a + b, 5);
}

TEST_F(MathTest, Multiplication) {
    EXPECT_EQ(a * b, 6);
}

```

### TEST_P() 宏（参数化测试）

如果要对不同输入值运行相同的测试逻辑，可以使用参数化测试   

```cpp
#include <gtest/gtest.h>

class AddTest : public ::testing::TestWithParam<std::tuple<int, int, int>> {};

TEST_P(AddTest, HandlesVariousInputs) {
    int a = std::get<0>(GetParam());
    int b = std::get<1>(GetParam());
    int expected = std::get<2>(GetParam());

    EXPECT_EQ(add(a, b), expected);
}

INSTANTIATE_TEST_SUITE_P(
    AddTestCases,
    AddTest,
    ::testing::Values(
        std::make_tuple(1, 2, 3),
        std::make_tuple(2, 3, 5),
        std::make_tuple(-1, -1, -2)
    )
);

```

### 断言与 EXPECT / ASSERT

**EXPECT_***：失败时继续执行后续代码  

**ASSERT_***：失败时终止当前测试函数  

\* 号可以是：EQ（等于），NE（不等于），LT（小于），LE（小于等于），GT（大于），GE（大于等于），TRUE，FALSE，FLOAT_EQ，DOUBLE_EQ，NEAR(val1, val2, abs_error)（约等于），THROW(statement, exception_type) 是否抛出指定类型的异常。