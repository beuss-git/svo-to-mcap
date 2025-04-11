set(MCAP_VERSION "2.0.0")

include(FetchContent)

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.29")
  set(DOWNLOAD_EXTRACT_TIMESTAMP_ARG "DOWNLOAD_EXTRACT_TIMESTAMP;TRUE")
else()
  set(DOWNLOAD_EXTRACT_TIMESTAMP_ARG "")
endif()
set(MCAP_TAG "releases/cpp/v${MCAP_VERSION}")
FetchContent_Declare(
  mcap
  URL https://github.com/foxglove/mcap/archive/refs/tags/${MCAP_TAG}.tar.gz
  ${DOWNLOAD_EXTRACT_TIMESTAMP_ARG}
)
FetchContent_MakeAvailable(mcap)
set(MCAP_INCLUDE_DIR ${mcap_SOURCE_DIR}/cpp/mcap/include/)
