cmake_minimum_required(VERSION 3.10)
project(xarm_utils_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

add_library(xarm_utils
  src/xarm_utils.cpp
)

set_target_properties(xarm_utils PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

target_include_directories(xarm_utils PUBLIC include)

ament_target_dependencies(xarm_utils
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
)

install(TARGETS
  xarm_utils
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(example_xarm_utils examples/example_xarm_utils.cpp)
ament_target_dependencies(example_xarm_utils
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
)
target_link_libraries(example_xarm_utils
  xarm_utils
)
install(TARGETS example_xarm_utils
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- Pythonバインディング(pybind11) ----------
add_library(xarm_utils_py MODULE src/pybind_module.cpp src/xarm_utils.cpp)
target_include_directories(xarm_utils_py PRIVATE
    include
    /usr/include/python3.10  # バージョンは合っているか確認！
)
target_link_libraries(xarm_utils_py PRIVATE xarm_utils)
set_target_properties(xarm_utils_py PROPERTIES PREFIX "")
include(GNUInstallDirs)
install(TARGETS xarm_utils_py DESTINATION ${CMAKE_INSTALL_LIBDIR}/python3.10/site-packages)
# ---------- install ----------
install(TARGETS xarm_utils DESTINATION lib/${PROJECT_NAME})
install(TARGETS example_xarm_utils DESTINATION lib/${PROJECT_NAME})

ament_package()
