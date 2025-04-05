cmake_minimum_required(VERSION 3.8)
project(stereo_proc)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
find_package(image_proc REQUIRED)

# Enable position independent code for all targets
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Build ELAS core library (static)
file(GLOB ELAS_SOURCES src/libelas/*.cpp)
add_library(elas STATIC ${ELAS_SOURCES})
target_include_directories(elas PUBLIC 
  include/libelas
)

# Build elas_matching library (shared)
add_library(elas_matching SHARED
  src/elas_matching/elas_matching.cpp
)
target_include_directories(elas_matching PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(elas_matching
  elas
  ${OpenCV_LIBRARIES}
)

# Install elas_matching library
install(TARGETS elas_matching
  LIBRARY DESTINATION lib
)

# Main stereo_proc library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/stereo_proc/stereo_proc.cpp
  src/stereo_proc/disp_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/stereo_proc
)

target_link_libraries(${PROJECT_NAME}
  elas_matching
  ${OpenCV_LIBRARIES}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Component registration
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "stereo_proc::DisparityNode"
  EXECUTABLE disp_node
)

# Export libraries
ament_export_libraries(elas_matching)

# Testing
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

ament_package()