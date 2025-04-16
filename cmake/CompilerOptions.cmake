# Enable warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Linker optimizations
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
  target_link_options(${PROJECT_NAME} PRIVATE -Wl,--gc-sections)
endif()

# Debugging settings
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g)
endif()

