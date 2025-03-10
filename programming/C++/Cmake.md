# CMakeLists.txt
## 指定 Cmake 最低版本
cmake_minimum_required(VERSION 3.4.1)   
## 设置工程名称
project(demo)  

## 设置编译器选择
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${CMAKE_CURRENT_SOURCE_DIR} -L${CMAKE_CURRENT_SOURCE_DIR}/libs -std=c++17 -O2 -g -Wall -Werror")  
**-std=c++17**: 使用 C++17 标准  
**-O2**: 进行中度优化  
**-g**: 使用调试信息  
**-Wall**:   
**-Werror**: 显示所有常见警告，并将警告视为错误  

更推荐使用 target_compile_options，可以控制每个目标的编译选项  
add_executable(my_program main.cpp)  
target_compile_options(my_program PRIVATE -std=c++17 -O2 -g -Wall -Werror)  

## 查找系统安装包 
find_package(Boost REQUIRED)  
find_package(Boost 1.7 REQUIRED)   
find_package(Boost COMPONENT system) # 补充 COMPONENT 的具体规则  
这个命令会查找系统上安装的包所在路径，执行这个包的配置文件，如 BoostConfig.cmake, BoostModule.cmake，得到包的头文件和库文件变量。  
### 模块模式
搜索路径依次为 CMAKE_MODULE_PATH，CMAKE_ROOT  
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR/cmake"})  
find_package(my_test MODULE REQUIRED)  
if (NOT my_test_FOUND)  
    message("my_test is not found")  
    message("build with debug mode")  
    message(WARNING "this is warnning message")  
    message(FATA_ERROR "this build has many error") # 会导致编译失败  
    message(STATUS "project source dir: ${PROJECT_SOURCE_DIR}") # 输出信息前边会有 -- 的前缀  
else()  
    xxx  
endif()  
### 配置搜索 
搜索顺序为 <packageName>_ROOT, CMAKE_PREFIX_PATH（如果安装了ROS1，这个变量会首先查找 ROS 安装的库），CMAKE_FRAMEWORK_PATH,CMAKE_APPBUNDLE_PATH,PATH
find_package(Eigen3 REQUIRED)  
set(yaml-cpp_DIR "${path_to_third_party}/yaml_cpp-1.12/lib/cmake/yaml-cpp")  #指定包路径，包名_DIR(配置文件 .cmake 所在路径)，包名_INCLUDE_DIRS(头文件路径), 包名_LIBRARIES（静动态库路径）  
find_package(yaml-cpp 1.12 REQUIRED NO_DEFAULT_PATH)  # 不使用系统安装的默认路径  

### 实践问题
工作中曾经遇到一个问题，docker 里的 Ubuntu 20.04 已经安装了 ROS 系统，ROS 自动安装了 pcl1.10 和 boost 1.7。但是 docker 里还通过源码安装了 PCL 1.8，Boost 1.65。程序里本来是不依赖 ROS 的，但是有个项目一开始点云数据只有 bag 包，所以只好使用 rosbag 来读 bag。CMakeLists 里就添加了 
find_package(Boost REQUIRED COMPONENTS system) 
find_package(catkin REQUIRED COMPONENTS  
    roscpp  
    rosbag  
    std_msgs  
)
catkin_package(CATKIN_DEPENDS roscpp rosbag std_msgs)  
结果编译的时候就报 ROS 需要 Boost 1.7，但是有一个 Boost 1.65，存在冲突。
然后把我就试着
1. 修改 find_package 指定 Boost 版本，不行  
2. LD_LIBRARY_PATH 添加 /usr/lib/x86_64-linux-gnu，不行  
3. 既然 ROS 已经给装了  Boost 1.7，那我就把源码安装的 1.65 删点好了。好家伙，PCL 1.8 依赖了 Boost 1.65...，那好吧，我重新装一下 PCL 好了，顺便升级一下版本，毕竟 1.8 版本在点云模型分割那块有 Bug，会 core dump，至少得 1.9.1 才没问题。好不容易把 1.9.1 的 PCL 装好了，程序运行的时候出了另一个很奇葩的问题。  
问题：自定义的 struct Box，里边有 EIGEN::Matrix cv::Vec 类型的变量，之前也没有 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 宏解决在 vector，map 里内存对齐的问题，当然之前跑是没问题的，不会报错。但是这次在 vector 里push_back的时候出内存分配的问题了，打印的vector size 出现超级大的一串数字。  
尝试：  
1. 使用 EIGEN_MAKE_ALIGEND_OPERATOR_NEW，调整 struct 变量顺序，把占用字节多的放前边，并没有用  
2. 不用 push_back 了，用 emplace，可以自动进行构造，size 确实打印正常了，但是后边还是出现了 double_free or corruption (out) 的错误。简直无语嘞。
3. 原来的docker是从同事那拿来的，想着会不会是因为环境太乱了，出现了不可预料的错误。我想我入职后也没有自己完整配置一遍环境，所以就自己重新下了个 noetic ROS 版本的docker，自己重头把环境配一遍。结果把，我还自己装了个 1.10.0 的 PCL，感知程序一执行，又报上边那个 vector size 很大的问题。。。
4. 最后想着，ROS 不是已经装了 PCL 了嘛，那把源码安装的 PCL 删点好了。还真行，不报错了。。。  

**过于底层了，实在是没搞清楚具体为什么就内存分配异常了。。。经过这次的教训，以后这环境还是尽可能不要整太多的版本，然后能用 docker 就用 docker，这要是物理机，那不还得重新装个 Ubuntu。。。**  
**不过我觉得吧，应该还是有其他的方法能解决的，毕竟环境中某个包有多个版本也是很正常的，网上也确实看到了一些使用不同版本包的方法，只不过我可能正好遇到了比较复杂且有其他未知的情况，所以没有找到具体的原因和更加便捷的方法**  


## 查找指定的库文件
### 查找单个库文件
find_library(libvar mymath ./mymath)  
### 安装全路径查找指定的文件
find_file(result myfile ./path)  # 没看懂怎么用。。。  
find_file(find_result NAMES myfile1 myfile2 PATHS . ./path)  
### 找到包含指定文件的目录
find_path(<var> name1 path1 path2) # 没看懂怎么用。。。  

## 设置包含的头文件目录
include_directories(  
    ${Boost_INCLUDE_DIRS}  
    ${CMAKE_CURRENT_SOURCE_DIR}/include  
    somepath/to/include  
)  

## 设置链接搜索目录
link_directories(  
    ${CMAKE_CURRENT_SOURCE_DIR}/libs
)

## 文件搜索
**搜索目录下所有 cpp 文件**：aux_source_directory(. SRC_LIST) # 有些工程会出现用 .cc 的情况，那这个应该是无法包含 .cc 文件的吧  ？  
**自定义搜索规则**：  
file(GLOB SRC_LIST "*.cpp" "src/*.cpp" "src/*.cc")  
file(GLOB_RECURSE COMMON_SOURCE "*.cpp") # 递归搜索  

## 静动态库生成
**生成静态库(默认)**：add_library(common util.cpp util2.cpp) / add_library(common STATIC util.cpp util2.cpp)  
**生成动态库**：add_library(common SHARED ${SRC_LIST} ${COMMON_SOURCE}) 

## 生成可执行文件与链接
add_executable(demo demo.cpp demo2.cpp)  
target_link_libraries(demo  
    ${Boost_LIBRARIES}  
    libface.so  
)

## 安装


## 常用变量
### 预定义变量
**PROJECT_SOURCE_DIR**：工程的根目录  
**PROJECT_BINARY_DIR**：运行 CMake 命令的目录，通常是 ${PROJECT_SOURCE_DIR}/build  
PROJECT_NAME：返回通过 project 命令定义的项目名称  
**CMAKE_CURRENT_SOURCE_DIR**：当前处理的 CMakeLists.txt 所在的路径  
CMAKE_CURRENT_BINARY_DIR：target 编译目录  
CMAKE_CURRENT_LIST_DIR：CMakeLists.txt 的完整路径  
CMAKE_CURRENT_LIST_LINE：当前所在的行  
CMAKE_MODULE_PATH：定义自己的 CMake 模块所在的路径，然后用 INCLUDE 命令来调用自己的模块  
EXECUATABLE_OUTPUT_PATH：重新定义目标二进制可执行文件的存放位置  
LIBRARY_OUTPUT_PATH：重新定义目标链接库文件的存放位置  
### 设置环境变量
$ENV{name}    # 使用环境变量  
set(ENV{name} value)    # 写入环境变量  

## 主要开关选项
BUILD_SHARED_LIBS：用来控制默认的库编译方式，如果不进行设置，又没有进行指定，默认编译生成静态库
set(BUILD_SHARED_LIBS ON)    # 默认生成动态库  

## 指定安装路径
set(CMAKE_INSTALL_PREFIX <install_path>)  
或执行命令行 set(CMAKE_INSTALL_PREFIX <install_path>)
