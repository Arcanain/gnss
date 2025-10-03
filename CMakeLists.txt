# CMakeの最低バージョンを指定
cmake_minimum_required(VERSION 3.8)
# プロジェクト（パッケージ）名を定義
project(gnss)

# C++コンパイラ用の警告オプションを設定
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存パッケージを検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # NavSatFixのために追加
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# C++のバージョンを17に設定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#================================================================
# ライブラリの作成
#================================================================
# ノードとコンポーネントのソースファイルからライブラリを作成します。
# これにより、コードが再利用しやすくなります。
add_library(gnss_library
  src/gnss_node.cpp
  src/gnss_component.cpp
)

# ライブラリが依存するパッケージを指定します。
ament_target_dependencies(gnss_library
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

# ライブラリがインクルードディレクトリを見つけられるように設定します。
target_include_directories(gnss_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#================================================================
# 実行可能ファイルの作成
#================================================================
# main関数を含むファイルから実行可能ファイルを作成します。
add_executable(gnss_node_executable src/gnss.cpp)

# 実行可能ファイルが、先ほど作成したライブラリに依存することを指定します。
target_link_libraries(gnss_node_executable PRIVATE gnss_library)

#================================================================
# インストール設定
#================================================================
# ビルドした実行可能ファイルとライブラリをインストール先にコピーする
# これで `ros2 run` コマンドで実行できる
install(TARGETS
  gnss_node_executable
  gnss_library
  DESTINATION lib/${PROJECT_NAME}
)

# includeディレクトリ（ヘッダーファイル）をインストール先にコピーする
install(DIRECTORY
  include/
  DESTINATION include
)

# (将来的に追加する場合の例)
# launchファイルをインストール先にコピーします。
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )　　　written by Gemini

#================================================================
# Lint（コード静的解析）設定
#================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# amentパッケージとして必要な最後のコマンド
ament_package()