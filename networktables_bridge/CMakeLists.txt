cmake_minimum_required(VERSION 3.5)
project(networktables_bridge)

# Default to C99.
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require that dependencies from package.xml be available.
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

## Create executables
set(${PROJECT_NAME}_nt_bridge_node_SRC
  src/nt_bridge.cpp
)

ament_auto_add_executable(nt_bridge_node
  ${${PROJECT_NAME}_nt_bridge_node_SRC}
)

# Explicitly link wpilib's ntcore dependencies
# TODO(Winston): Find a better way to do this
target_link_libraries(nt_bridge_node ntcore wpinet wpiutil wpimath)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Create & install ament package.
ament_auto_package(
  INSTALL_TO_SHARE 
  launch
  # param
)
