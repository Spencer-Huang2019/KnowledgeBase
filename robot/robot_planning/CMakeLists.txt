include_directories(
    ./include
)

file(GLOB_RECURSE SRC_LIST "src/*.cpp")

add_library(planning ${SRC_LIST})

add_executable(robot_planning planning_main.cpp)
target_link_libraries(robot_planning planning)