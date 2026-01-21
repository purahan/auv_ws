cmake_minimum_required(VERSION 3.8)
project(nemo_description)

# Add compile options (good practice for C++ packages, even if unused here)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required for all ROS 2 packages
find_package(ament_cmake REQUIRED)

# Optional: Declare install locations for robot description files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Enable linting if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)      # Skip cpplint if not needed
  set(ament_cmake_copyright_FOUND TRUE)    # Skip copyright
  ament_lint_auto_find_test_dependencies()
endif()

# Export the package
ament_package()