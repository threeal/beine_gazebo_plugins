cmake_minimum_required(VERSION 3.5)

project(beine_gazebo_plugins)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(beine_cpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(keisan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)

install(DIRECTORY "include" DESTINATION ".")

set(PLUGINS
  "src/legs_plugin.cpp"
  "src/status_plugin.cpp")

foreach(PLUGIN ${PLUGINS})
  get_filename_component(TARGET ${PLUGIN} NAME_WE)

  add_library("beine_${TARGET}" SHARED ${PLUGIN})

  target_include_directories("beine_${TARGET}" PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies("beine_${TARGET}"
    beine_cpp gazebo_ros keisan rclcpp)

  target_link_libraries("beine_${TARGET}" Qt5::Core)

  install(TARGETS "beine_${TARGET}"
    DESTINATION "share/${PROJECT_NAME}/plugins")
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  beine_cpp gazebo_ros keisan rclcpp)

ament_export_include_directories("include")

ament_package()
