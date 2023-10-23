cmake_minimum_required(VERSION 3.12)

# プロジェクト名設定
project(PointCraft)

# 必須ライブラリの存在チェック
find_package(glfw3 REQUIRED)

# glad周り
include_directories(include/glad/include)
add_subdirectory(include/glad)

# コンパイルオプション
add_compile_options(-O2 -Wall -I /usr/include/eigen3)

# 実行ファイル
add_executable(PointCraft ./src/main.cpp)

# 実行ファイルにリンクするライブラリの指定
target_link_libraries(PointCraft glad glfw ${CMAKE_DL_LIBS})

# c++17を使用
set_property(TARGET PointCraft PROPERTY CXX_STANDARD 17)
