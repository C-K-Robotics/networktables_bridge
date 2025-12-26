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
find_package(ament_cmake_python REQUIRED)
# find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
# find_package(pybind11 CONFIG REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# set(${PROJECT_NAME}_HEADER
#   include/${PROJECT_NAME}/nt_bridge.hpp
# )

# set(${PROJECT_NAME}_SRC
#   src/nt_bridge.cpp
# )

# ## Create this project as a library.
# ament_auto_add_library(${PROJECT_NAME} SHARED
#   ${${PROJECT_NAME}_HEADER}
#   ${${PROJECT_NAME}_SRC}
# )

# pybind11_add_module(nt_pubsub src/nt_pubsub_py.cpp)
# target_link_libraries(nt_pubsub PRIVATE ${PROJECT_NAME})
# install(TARGETS nt_pubsub
#   LIBRARY DESTINATION local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages
# )

# ## Create this project as a Python library.
# ament_python_install_package(${PROJECT_NAME})

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
