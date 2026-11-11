cmake_minimum_required(VERSION 3.16)
project(yopo_hmm_runner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "OHOS")
  message(FATAL_ERROR "Use the OpenHarmony Native SDK toolchain")
endif()

if(NOT TCIM_RUNTIME_PATH)
  message(FATAL_ERROR "TCIM_RUNTIME_PATH must contain include/tcim and lib")
endif()

add_executable(yopo_hmm_runner main.cc)
target_include_directories(yopo_hmm_runner PRIVATE
  "${TCIM_RUNTIME_PATH}/include")
target_link_directories(yopo_hmm_runner PRIVATE
  "${TCIM_RUNTIME_PATH}/lib")
target_link_libraries(yopo_hmm_runner PRIVATE
  tcim_runtime_lite c++ c++abi)
set_target_properties(yopo_hmm_runner PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN/lib")
