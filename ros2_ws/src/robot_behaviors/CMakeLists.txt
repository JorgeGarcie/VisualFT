cmake_minimum_required(VERSION 3.5)
project(floating_scan)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fastrtps 2.6.7 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cppzmq QUIET)
if(NOT cppzmq_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(ZMQ libzmq)
  endif()
endif()

# RDK headers and static library
set(RDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../flexiv_ros2/flexiv_hardware/rdk")

# Use RDK's bundled spdlog
set(spdlog_DIR "${RDK_DIR}/thirdparty/cloned/spdlog/build")
find_package(spdlog REQUIRED)

# Platform-specific RDK static library
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  set(RDK_STATIC_LIBRARY "${RDK_DIR}/lib/libflexiv_rdk.x86_64-linux-gnu.a")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  set(RDK_STATIC_LIBRARY "${RDK_DIR}/lib/libflexiv_rdk.aarch64-linux-gnu.a")
else()
  message(FATAL_ERROR "Unsupported processor: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# ── ArmCommander static library ──────────────────────────────────
add_library(arm_commander STATIC
  src/arm_commander.cpp
  src/config.cpp
  src/scan_config.cpp
  src/teleop_config.cpp
)

target_include_directories(arm_commander PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${RDK_DIR}/include
)

target_link_libraries(arm_commander
  ${RDK_STATIC_LIBRARY}
  Eigen3::Eigen
  spdlog::spdlog
  fastrtps
  yaml-cpp
)

# ── floating_scan executable ─────────────────────────────────────
add_executable(floating_scan src/floating_scan.cpp)
target_link_libraries(floating_scan arm_commander)

# ── scan_controller executable ───────────────────────────────────
add_executable(scan_controller src/scan_controller.cpp)
target_link_libraries(scan_controller arm_commander)

# ── teleop executable (requires zmq) ─────────────────────────────
if(cppzmq_FOUND OR ZMQ_FOUND)
  add_executable(teleop src/teleop.cpp)
  if(cppzmq_FOUND)
    target_link_libraries(teleop arm_commander cppzmq)
  else()
    target_include_directories(teleop PRIVATE ${ZMQ_INCLUDE_DIRS})
    target_link_libraries(teleop arm_commander ${ZMQ_LIBRARIES})
  endif()
  install(TARGETS teleop DESTINATION lib/${PROJECT_NAME})
  install(PROGRAMS scripts/zmq_raw_publisher.py DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "ZMQ not found -- skipping teleop executable")
endif()

# ── Install ──────────────────────────────────────────────────────
install(TARGETS floating_scan scan_controller DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(TARGETS arm_commander
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
