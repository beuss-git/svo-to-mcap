FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(yaml-cpp)

get_target_property(YAML_CPP_INCLUDE_DIR yaml-cpp::yaml-cpp INTERFACE_INCLUDE_DIRECTORIES)
