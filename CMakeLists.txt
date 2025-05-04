cmake_minimum_required(VERSION 3.5)
project(svo_to_mcap)
#set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=*")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "-g")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
  message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()


include(cmake/CompilerWarnings.cmake)

macro(beuss_supports_sanitizers)
  if((CMAKE_CXX_COMPILER_ID MATCHES ".*Clang.*" OR CMAKE_CXX_COMPILER_ID MATCHES ".*GNU.*") AND NOT WIN32)
    set(SUPPORTS_UBSAN ON)
  else()
    set(SUPPORTS_UBSAN OFF)
  endif()

  if((CMAKE_CXX_COMPILER_ID MATCHES ".*Clang.*" OR CMAKE_CXX_COMPILER_ID MATCHES ".*GNU.*") AND WIN32)
    set(SUPPORTS_ASAN OFF)
  else()
    set(SUPPORTS_ASAN ON)
  endif()
endmacro()

beuss_supports_sanitizers()

option(beuss_ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" ${SUPPORTS_ASAN})
option(beuss_ENABLE_SANITIZER_LEAK "Enable leak sanitizer" OFF)
option(beuss_ENABLE_SANITIZER_UNDEFINED "Enable undefined sanitizer" ${SUPPORTS_UBSAN})
option(beuss_ENABLE_SANITIZER_THREAD "Enable thread sanitizer" OFF)
option(beuss_ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" OFF)

option(beuss_ENABLE_CACHE "Enable ccache" OFF)
option(beuss_WARNINGS_AS_ERRORS "Treat warnings as errors" ON)

add_library(beuss_warnings INTERFACE)
add_library(beuss_options INTERFACE)
beuss_set_project_warnings(
    beuss_warnings
    ${beuss_WARNINGS_AS_ERRORS}
    ""
    ""
    ""
    "")

include(cmake/Sanitizers.cmake)
beuss_enable_sanitizers(
    beuss_options
    ${beuss_ENABLE_SANITIZER_ADDRESS}
    ${beuss_ENABLE_SANITIZER_LEAK}
    ${beuss_ENABLE_SANITIZER_UNDEFINED}
    ${beuss_ENABLE_SANITIZER_THREAD}
    ${beuss_ENABLE_SANITIZER_MEMORY})

include(cmake/mcap.cmake)
include(cmake/yaml-cpp.cmake)
include(cmake/magic_enum.cmake)
include(cmake/fmt.cmake)

find_package(ZED 5 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(${PROJECT_NAME}
  src/utils/zed_to_ros.cpp
  src/mcap/mcap_impl.cpp
  src/mcap/mcap_writer.cpp
  src/camera_manager.cpp
  src/config.cpp
  src/converter.cpp
  src/main.cpp
)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MCAP_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

target_compile_options(${PROJECT_NAME} PRIVATE -fdiagnostics-color=always)


if (LINK_SHARED_ZED)
  set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
  set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

ament_target_dependencies(${PROJECT_NAME} std_msgs sensor_msgs rclcpp)
target_link_libraries(${PROJECT_NAME} ${ZED_LIBS} ${OpenCV_LIBRARIES} zstd yaml-cpp::yaml-cpp magic_enum::magic_enum fmt::fmt beuss_warnings beuss_options)

if(beuss_ENABLE_CACHE)
  include(cmake/Cache.cmake)
  beuss_enable_cache()
endif()

ament_package()
