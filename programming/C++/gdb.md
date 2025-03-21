gdb 调试工具还是用得比较少的，很多时候还都是通过打印来调试了。。。之前尝试过打断点之类的方法，但是最后还是觉得太麻烦了没有用，特别是每次遇到 core dumped 的时候，感觉 gdb 方法也没法直接告诉我在哪个函数或哪一行。。。也可能是我使用的方法不对吧。不过还是先记录一下基础的用法。

# 基础设置
1. 编译时添加调试信息，添加选项 -g  
g++ -g -o my_program my_program.cpp
cmakelists.txt 里的设置：  
set(CMAKE_BUILD_TYPE Debug)  
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")  
2. 启动 gdb
gdb ./my_program
3. 运行程序
(gdb) run
或可传递参数
(gdb) run arg1 arg2
# 常用命令
## 设置断点
按函数设置断点  
(gdb) break function_name
按文件和行号设置断点  
(gdb) break filename.cpp:line_number
按条件设置断点  
(gdb) break function_name if condition
例如：  
(gdb) break my_function if x > 10
列出所有断点  
(gdb) info breakpoints
删除断点  
(gdb) delete breakpoint_number
## 单步调试
下一步执行（不进入函数内部）  
(gdb) next  
进入函数内部  
(gdb) step  
跳出当前函数  
(gdb) finish  
继续运行程序（到下一个断点或结束）  
(gdb) continue  
跳过若干行  
(gdb) jump line_number  
## 查看变量
打印变量值  
(gdb) print variable_name  
例如：
(gdb) print x  
打印内存地址  
(gdb) print &variable_name  
查看局部变量  
(gdb) info locals  
查看全局变量  
(gdb) info variables  
持续监视变量值  
(gdb) display variable_name  
取消监视  
(gdb) undisplay display_number  
动态修改程序  
(gdb) set variable variable_name = new_value  
例如：  
(gdb) set variable x = 42  
## 启动程序到特定位置  
直接运行到某个断点或行    
(gdb) start  
(gdb) break main  
(gdb) run  
调试动态库  
(gdb) set solib-search-path /path/to/your/lib  