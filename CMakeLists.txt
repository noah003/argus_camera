cmake_minimum_required(VERSION 3.5)
project(argus_camera)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 11)

find_package(TegraMM REQUIRED)
find_package(OpenCV REQUIRED)

if(${TegraMM_FOUND})
  message(STATUS "Found Tegra Multimedia API")
else()
  message(STATUS "Could not find Tegra Multimedia API")
endif()

add_library(argus_camera SHARED
  src/ArgusCamera.cpp
  ${TegraMM_COMMON_SOURCES}
)

#target_link_libraries(argus_camera ${TegraMM_LIBRARIES} /usr/lib/aarch64-linux-gnu/tegra/libnvargus.so)
#set_target_properties(argus_camera PROPERTIES PUBLIC_HEADER 
#  "src/ArgusCamera.hpp"
#)

add_executable(test_argus_camera
  src/ArgusCamera.cpp
  src/test.cpp
  ${TegraMM_COMMON_SOURCES}
)
target_include_directories(test_argus_camera
  PRIVATE
  ${OpenCV_INCLUDE_DIRECTORIES}
)
target_link_libraries(test_argus_camera 
  ${TegraMM_LIBRARIES}
  ${OpenCV_LIBRARIES}
  /usr/lib/aarch64-linux-gnu/tegra/libnvargus.so
)

install(TARGETS argus_camera
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static
  PUBLIC_HEADER DESTINATION include/argus_camera)


# TESTS

if (BUILD_TESTS)
  find_package(GTest REQUIRED)

  add_executable(TestArgusCamera test/TestArgusCamera.cpp)
  target_link_libraries(TestArgusCamera gtest argus_camera)
endif()
