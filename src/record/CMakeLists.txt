cmake_minimum_required(VERSION 3.16)
project(record LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# The Rerun C++ SDK is installed at /opt/rerun_sdk_install by the Dockerfile.
# Override RERUN_SDK_DIR (env var) to point at your own install tree if needed.
if(DEFINED ENV{RERUN_SDK_DIR})
  set(RERUN_SDK_DIR "$ENV{RERUN_SDK_DIR}")
else()
  set(RERUN_SDK_DIR "/opt/rerun_sdk_install")
endif()

foreach(_lib librerun_sdk.a librerun_c__linux_x64.a libarrow.a libarrow_bundled_dependencies.a)
  if(NOT EXISTS "${RERUN_SDK_DIR}/lib/${_lib}")
    message(FATAL_ERROR
      "Rerun SDK file missing: ${RERUN_SDK_DIR}/lib/${_lib}\n"
      "The Dockerfile installs the Rerun C++ SDK to /opt/rerun_sdk_install — rebuild the image, "
      "or set the RERUN_SDK_DIR environment variable to your own install tree.")
  endif()
endforeach()

add_library(rerun_c STATIC IMPORTED)
set_target_properties(rerun_c PROPERTIES
  IMPORTED_LOCATION "${RERUN_SDK_DIR}/lib/librerun_c__linux_x64.a"
)

add_library(arrow STATIC IMPORTED)
set_target_properties(arrow PROPERTIES
  IMPORTED_LOCATION "${RERUN_SDK_DIR}/lib/libarrow.a"
)

add_library(arrow_bundled STATIC IMPORTED)
set_target_properties(arrow_bundled PROPERTIES
  IMPORTED_LOCATION "${RERUN_SDK_DIR}/lib/libarrow_bundled_dependencies.a"
)

add_library(rerun_sdk STATIC IMPORTED)
set_target_properties(rerun_sdk PROPERTIES
  IMPORTED_LOCATION "${RERUN_SDK_DIR}/lib/librerun_sdk.a"
  INTERFACE_INCLUDE_DIRECTORIES "${RERUN_SDK_DIR}/include"
  INTERFACE_LINK_LIBRARIES "rerun_c;arrow;arrow_bundled"
)

message(STATUS "Using Rerun SDK: ${RERUN_SDK_DIR}")

include_directories(include)

add_executable(record_node
  src/record_node.cpp
  src/rerun_interface.cpp
)

ament_target_dependencies(record_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(record_node rerun_sdk)

install(TARGETS
  record_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
