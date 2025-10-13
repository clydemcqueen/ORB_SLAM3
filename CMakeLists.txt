cmake_minimum_required(VERSION 3.14.4)
project(ORB_SLAM3)

message(STATUS "${PROJECT_NAME} build type: " ${CMAKE_BUILD_TYPE})

# Extract vocabulary to build directory
set(VOCABULARY_ARCHIVE "${PROJECT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz")
set(VOCABULARY_OUTPUT_DIR "${CMAKE_BINARY_DIR}/Vocabulary")
set(VOCABULARY_FILE "${VOCABULARY_OUTPUT_DIR}/ORBvoc.txt")

if(EXISTS ${VOCABULARY_ARCHIVE})
  message(STATUS "Extracting vocabulary file...")
  file(MAKE_DIRECTORY ${VOCABULARY_OUTPUT_DIR})

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${VOCABULARY_ARCHIVE}
    WORKING_DIRECTORY ${VOCABULARY_OUTPUT_DIR}
    RESULT_VARIABLE TAR_RESULT
  )

  if(TAR_RESULT EQUAL 0)
    message(STATUS "Vocabulary extracted to: ${VOCABULARY_FILE}")
  else()
    message(FATAL_ERROR "Failed to extract vocabulary archive")
  endif()
else()
  message(WARNING "Vocabulary archive not found at: ${VOCABULARY_ARCHIVE}")
endif()

add_subdirectory(Thirdparty/Sophus)
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(OpenCV 4.4)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

message(STATUS "OpenCV version: " ${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)

# Disable some warnings for now
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion-null")

add_library(${PROJECT_NAME} SHARED)

target_sources(${PROJECT_NAME}
  PRIVATE
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc
)

# Specify include directories, be careful about build vs install
target_include_directories(${PROJECT_NAME}
  PUBLIC
  # For build:
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>  # Handles includes of the form: "Thirdparty/..."
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/CameraModels>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/Sophus>

  # For install:
  $<INSTALL_INTERFACE:.>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/CameraModels>
  $<INSTALL_INTERFACE:Thirdparty/Sophus>

  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  DBoW2
  g2o
  -lboost_serialization
  -lcrypto
)

# Testing
enable_testing()
add_subdirectory(Thirdparty/googletest)

add_executable(pinhole_test test/test_pinhole.cc)
target_link_libraries(pinhole_test
  PRIVATE
  gtest_main
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)
target_include_directories(pinhole_test
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)
include(GoogleTest)
gtest_discover_tests(pinhole_test)
