cmake_minimum_required(VERSION 3.8)
project(stereo_proc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(image_proc REQUIRED)

# PIC default ON
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# build elas 
file(GLOB ELAS_SOURCES src/libelas/*.cpp)
add_library(elas STATIC ${ELAS_SOURCES})
target_include_directories(elas PUBLIC 
  include/libelas
)

# added elas_matching wrapper
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/stereo_proc/stereo_proc.cpp
  src/stereo_proc/disp_node.cpp
  src/stereo_proc/elas_matching.cpp
)

# link elas
target_link_libraries(${PROJECT_NAME}
  elas 
  ${OpenCV_LIBRARIES}
)

# rclcpp component registration
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "stereo_proc::DisparityNode"
  EXECUTABLE disp_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
  endif()
  ament_add_pytest_test("test_disparity_node" test/test_disparity.py
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch)