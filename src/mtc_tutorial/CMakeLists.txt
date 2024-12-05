cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mtc_basic src/mtc_basic.cpp)
add_executable(mtc_elev src/mtc_elev.cpp)
add_executable(mtc_obs src/mtc_obs.cpp)
add_executable(mtc_obs_elev src/mtc_obs_elev.cpp)

ament_target_dependencies(mtc_basic "moveit_task_constructor_core" "rclcpp")
ament_target_dependencies(mtc_elev "moveit_task_constructor_core" "rclcpp")
ament_target_dependencies(mtc_obs "moveit_task_constructor_core" "rclcpp")
ament_target_dependencies(mtc_obs_elev "moveit_task_constructor_core" "rclcpp")

# target_include_directories(mtc_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   mtc_basic
#   mtc_elev
#   mtc_obs
#   mtc_obs_elev
#   "moveit_task_constructor_core"
#   "rclcpp"
# )

install(TARGETS
  mtc_basic
  mtc_elev
  mtc_obs
  mtc_obs_elev
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
