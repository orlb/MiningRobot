cmake_minimum_required(VERSION 3.8)
project(aurora_core)

include_directories(include)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(ouster_ros REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

add_executable(aurora_node_core src/aurora_node_core.cpp)
ament_target_dependencies(aurora_node_core ouster_ros)
target_link_libraries(aurora_node_core Eigen3::Eigen)

# Mark executables for installation
install(TARGETS aurora_node_core
    DESTINATION lib/${PROJECT_NAME})

ament_package()
