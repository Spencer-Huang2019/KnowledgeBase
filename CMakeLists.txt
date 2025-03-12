cmake_minimum_required(VERSION 3.1.0)
project(knowledge_base)

set(BUILD_SHARED_LIBS ON)

include_directories(
    ${PROJECT_SOURCE_DIR}/common/include
)

add_subdirectory(robot)