cmake_minimum_required(VERSION 3.5)
project(robot_behaviors)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(arm_commander REQUIRED)
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

# RDK path (needed for spdlog and RDK headers transitively included by arm_commander)
set(RDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../flexiv_ros2/flexiv_hardware/rdk")
set(spdlog_DIR "${RDK_DIR}/thirdparty/cloned/spdlog/build")
find_package(spdlog REQUIRED)

include_directories(${RDK_DIR}/include)

# ── App-specific config library ──────────────────────────────────
add_library(behavior_configs STATIC
  src/scan_config.cpp
  src/teleop_config.cpp
)

target_include_directories(behavior_configs PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(behavior_configs PUBLIC
  arm_commander::arm_commander
  Eigen3::Eigen
  spdlog::spdlog
  yaml-cpp
)

# ── floating_scan executable ─────────────────────────────────────
add_executable(floating_scan src/floating_scan.cpp)
target_link_libraries(floating_scan arm_commander::arm_commander spdlog::spdlog)

# ── massage executable ───────────────────────────────────────────
add_executable(massage src/massage.cpp)
target_link_libraries(massage arm_commander::arm_commander spdlog::spdlog)

# ── scan_controller executable ───────────────────────────────────
add_executable(scan_controller src/scan_controller.cpp)
target_link_libraries(scan_controller behavior_configs)

# ── teleop executable (requires zmq) ─────────────────────────────
if(cppzmq_FOUND OR ZMQ_FOUND)
  add_executable(teleop src/teleop.cpp)
  if(cppzmq_FOUND)
    target_link_libraries(teleop behavior_configs cppzmq)
  else()
    target_include_directories(teleop PRIVATE ${ZMQ_INCLUDE_DIRS})
    target_link_libraries(teleop behavior_configs ${ZMQ_LIBRARIES})
  endif()
  install(TARGETS teleop DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "ZMQ not found -- skipping teleop executable")
endif()

# ── Install ──────────────────────────────────────────────────────
install(TARGETS floating_scan massage scan_controller DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/vr_server.py DESTINATION lib/${PROJECT_NAME})

ament_package()
