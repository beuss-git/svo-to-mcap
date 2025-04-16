cmake_minimum_required(VERSION 3.5)
project(svo_to_mcap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
  message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

include(cmake/mcap.cmake)
include(cmake/yaml-cpp.cmake)
include(cmake/foxglove-proto.cmake)
include(cmake/magic_enum.cmake)
include(cmake/fmt.cmake)

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)


include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${MCAP_INCLUDE_DIR})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})


add_executable(${PROJECT_NAME}
  src/main.cpp
  src/config.cpp
  src/utils/thread_pool.cpp
  src/camera_manager.cpp
  src/mcap/mcap_impl.cpp
  src/mcap/mcap_writer.cpp
  src/foxglove/BuildFileDescriptorSet.cpp
)

if (LINK_SHARED_ZED)
  set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
  set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

target_link_libraries(${PROJECT_NAME} ${ZED_LIBS} ${OpenCV_LIBRARIES} zstd yaml-cpp::yaml-cpp foxglove_proto_lib magic_enum::magic_enum fmt::fmt)

