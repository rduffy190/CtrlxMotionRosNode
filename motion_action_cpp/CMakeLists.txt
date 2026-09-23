cmake_minimum_required(VERSION 3.8)
project(motion_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Useful for IntelliSense / clangd - lets the editor see the real include paths,
# including whichever Data Layer branch below actually resolves.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# NOTE: the ctrlX SDK C++ samples build with -fno-rtti. Do NOT copy that here -
# rclcpp requires RTTI. Linking the shared libcomm_datalayer.so keeps the two
# apart.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(axis_status_msg REQUIRED)
find_package(ctrlx_motion_action REQUIRED)

# ---------------------------------------------------------------------------
# ctrlX Data Layer
#
# Preferred: the ctrlx-datalayer .deb is installed, which ships a pkg-config
# file (headers in /usr/include and /usr/include/comm/datalayer/c, plus the
# bundled flatbuffers headers in /usr/include/flatbuffers, library in
# /usr/lib/<triplet>). See scripts/install-ctrlx-datalayer.sh in the SDK.
#
# Fallback: point at an unpacked SDK archive with
#   colcon build --cmake-args -DCTRLX_SDK_DIR=$HOME/ctrlx-automation-sdk
# ---------------------------------------------------------------------------
find_package(PkgConfig QUIET)
set(CTRLX_SDK_DIR "" CACHE PATH "Root of an unpacked ctrlx-automation-sdk archive")

if(NOT CTRLX_SDK_DIR AND PkgConfig_FOUND)
  pkg_check_modules(CTRLX_DATALAYER QUIET ctrlx-datalayer)
endif()

if(CTRLX_DATALAYER_FOUND)
  message(STATUS "ctrlX Data Layer: pkg-config (${CTRLX_DATALAYER_VERSION})")
  set(CTRLX_INCLUDE_DIRS ${CTRLX_DATALAYER_INCLUDE_DIRS})
  set(CTRLX_LIBRARY_DIRS ${CTRLX_DATALAYER_LIBRARY_DIRS})
  set(CTRLX_LIBRARIES ${CTRLX_DATALAYER_LIBRARIES})
elseif(CTRLX_SDK_DIR)
  message(STATUS "ctrlX Data Layer: SDK tree at ${CTRLX_SDK_DIR}")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(CTRLX_PLATFORM "ubuntu24-gcc-aarch64")
  else()
    set(CTRLX_PLATFORM "ubuntu24-gcc-x64")
  endif()
  set(CTRLX_INCLUDE_DIRS
    ${CTRLX_SDK_DIR}/include
    ${CTRLX_SDK_DIR}/include/comm.datalayer)
  set(CTRLX_LIBRARY_DIRS ${CTRLX_SDK_DIR}/lib/comm.datalayer/${CTRLX_PLATFORM}/release)
  set(CTRLX_LIBRARIES comm_datalayer)
else()
  message(FATAL_ERROR
    "ctrlX Data Layer not found.\n"
    "Either install the ctrlx-datalayer debian package, or configure with "
    "-DCTRLX_SDK_DIR=/path/to/ctrlx-automation-sdk")
endif()

# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
add_executable(ctrlx_motion_node
  src/ctrlx_motion_node.cpp
  src/ctrlx_axis_itf.cpp
  src/ctrlx_status_itf.cpp
)

target_include_directories(ctrlx_motion_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${CTRLX_INCLUDE_DIRS}
)

target_link_directories(ctrlx_motion_node PRIVATE ${CTRLX_LIBRARY_DIRS})

ament_target_dependencies(ctrlx_motion_node
  rclcpp
  rclcpp_action
  axis_status_msg
  ctrlx_motion_action
)

target_link_libraries(ctrlx_motion_node
  ${CTRLX_LIBRARIES}
  pthread
  systemd
  ssl
  crypto
)

install(TARGETS ctrlx_motion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# The vendored ctrlX MOTION flatbuffer schemas under
# include/motion_action_cpp/fbs/ are header-only and are picked up by the
# target_include_directories entry above.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
