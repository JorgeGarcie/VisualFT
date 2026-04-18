cmake_minimum_required(VERSION 3.5)
project(arm_commander)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fastrtps 2.6.7 REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# ── arm_commander static library ─────────────────────────────────
add_library(arm_commander STATIC
  src/arm_commander.cpp
  src/config.cpp
)

target_include_directories(arm_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${RDK_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(arm_commander PUBLIC
  ${RDK_STATIC_LIBRARY}
  Eigen3::Eigen
  spdlog::spdlog
  fastrtps
  yaml-cpp
)

# ── Install ──────────────────────────────────────────────────────
install(DIRECTORY include/ DESTINATION include)

install(TARGETS arm_commander
  EXPORT arm_commander_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(arm_commander_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
