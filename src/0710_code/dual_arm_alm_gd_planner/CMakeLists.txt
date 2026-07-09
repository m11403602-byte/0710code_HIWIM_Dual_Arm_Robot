cmake_minimum_required(VERSION 3.16)
project(dual_arm_alm_gd_planner)

# ===== Release + 最大優化 =====
#   單一 .so 架構: 所有 .cpp (含 planner_manager) 編進同一個 .so。
#   ⚠ 不可用 -march=native: 會讓 Eigen 啟用 AVX (32-byte 對齊),
#      與 MoveIt 標準編譯 (16-byte 對齊) 在傳遞 Eigen 物件時不一致,
#      碰撞路徑 regenerate 重建軌跡後於 __libc_free 崩潰。
#   ✅ 實測: -O3 -DNDEBUG (不含 march=native) 可正常運作, 速度也夠快。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3 -DNDEBUG)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 依賴 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# =====================================================================
# 單一 .so: 核心演算法 + MoveIt 插件全包在一起
#   gd_solver / avoidance_system / data_io = 核心 (純 Eigen)
#   planner_manager = MoveIt 插件介面
#   全部編進同一個 .so, Eigen 物件不跨 .so 邊界
# =====================================================================
add_library(${PROJECT_NAME} SHARED
  src/gd_solver.cpp
  src/avoidance_system.cpp
  src/data_io.cpp
  src/planner_manager.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp pluginlib moveit_core moveit_msgs Eigen3
)

# 註冊 plugin 描述檔到 moveit_core
pluginlib_export_plugin_description_file(moveit_core dual_arm_alm_gd_planner.xml)

# =====================================================================
# 獨立執行檔 + 測試程式 (連同一個 .so, 照常呼叫 AvoidanceSystem)
# =====================================================================
# ===== 安裝 =====
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(moveit_core pluginlib rclcpp moveit_msgs Eigen3)
ament_package()
