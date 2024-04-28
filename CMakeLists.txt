cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(JSON_ImplicitConversions OFF) # deactivate implicit type conversion

project(mission_control_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)  # JSON library
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(common_package REQUIRED)

include_directories(include)

#  Mission File Lib
add_library(mission_file_lib SHARED
  src/mission_definition_file.cpp)
ament_export_libraries(mission_file_lib)
ament_export_include_directories(include)
ament_target_dependencies(mission_file_lib common_package nlohmann_json)

install (
  DIRECTORY include/
  DESTINATION include
)

install (TARGETS mission_file_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mission Control Node
add_executable(mission_control_node
                src/mission_definition_file.cpp
                src/basics.cpp
                src/modes.cpp
                src/callbacks.cpp
                src/mission_control.cpp
                src/main.cpp)
ament_target_dependencies(mission_control_node rclcpp interfaces common_package nlohmann_json)

install(TARGETS
  mission_control_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
