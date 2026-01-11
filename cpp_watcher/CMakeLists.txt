cmake_minimum_required(VERSION 3.11)
project(ros_edge_hud_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 自动拉取 FTXUI 库
include(FetchContent)

FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/FTXUI
  GIT_TAG v5.0.0
)

FetchContent_MakeAvailable(ftxui)

# 添加主程序
add_executable(watcher src/main.cpp src/toolbox.cpp)

# 链接库
target_link_libraries(watcher
  PRIVATE ftxui::screen
  PRIVATE ftxui::dom
  PRIVATE ftxui::component
  PRIVATE stdc++fs # 用于文件系统操作
  PRIVATE pthread  # 用于多线程
)
