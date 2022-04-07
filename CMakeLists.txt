cmake_minimum_required(VERSION 3.20)

get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
message("Controller dir: ${PROJECT}")
project(${PROJECT})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# c++14 - to be compatible with g2o
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra)
endif()

add_subdirectory(slam_pipeline)

set(WEBOTS_DIR "WEBOTS_DIR-NOTFOUND" CACHE PATH "WeBots installation path")
if (EXISTS "${WEBOTS_DIR}")
  message("Onnx found at ${WEBOTS_DIR}")
else()
  message(FATAL_ERROR "WeBots was not found, please set the WEBOTS_DIR parameter")
endif()

set(ONNXRUNTIME_DIR "ONNXRUNTIME_DIR-NOTFOUND" CACHE PATH "ONNX runtime installation path")
if (EXISTS "${ONNXRUNTIME_DIR}")
  message("Onnx found at ${ONNXRUNTIME_DIR}")
  set(ONNXRUNTIME_INCLUDE_DIRS ${ONNXRUNTIME_DIR}/include)
  set(ONNXRUNTIME_LIBRARY_DIRS ${ONNXRUNTIME_DIR}/lib)
  if (MSVC)
    file(GLOB ONNXRUNTIME_LIBRARIES "${ONNXRUNTIME_LIBRARY_DIRS}/*.lib")
  endif()
else()
  message(FATAL_ERROR "Onnx was not found, please set the ONNXRUNTIME_DIR parameter")
endif()


find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


set(SOURCES 
    src/main.cpp
    src/dnnfeaturematcher.h
    src/dnnfeaturematcher.cpp
    src/featurematcher.h
    src/featurematcher.cpp
)

if (WIN32)
  set(CONTROLLER_SOURCES 
    ${WEBOTS_DIR}/src/controller/cpp/Accelerometer.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Altimeter.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Brake.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Camera.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Compass.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Connector.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Device.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Display.cpp
    ${WEBOTS_DIR}/src/controller/cpp/DistanceSensor.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Emitter.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Field.cpp
    ${WEBOTS_DIR}/src/controller/cpp/GPS.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Gyro.cpp
    ${WEBOTS_DIR}/src/controller/cpp/InertialUnit.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Joystick.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Keyboard.cpp
    ${WEBOTS_DIR}/src/controller/cpp/LED.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Lidar.cpp
    ${WEBOTS_DIR}/src/controller/cpp/LightSensor.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Motion.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Motor.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Mouse.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Node.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Pen.cpp
    ${WEBOTS_DIR}/src/controller/cpp/PositionSensor.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Radar.cpp
    ${WEBOTS_DIR}/src/controller/cpp/RangeFinder.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Receiver.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Robot.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Skin.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Speaker.cpp
    ${WEBOTS_DIR}/src/controller/cpp/Supervisor.cpp
    ${WEBOTS_DIR}/src/controller/cpp/TouchSensor.cpp
  )
  source_group(ControllerLib FILES ${CONTROLLER_SOURCES})
else()
  set(CONTROLLER_SOURCES )
endif()

# Link libraries
link_directories(${ONNXRUNTIME_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
message("Boost library dir: ${Boost_LIBRARY_DIRS}")

link_directories(${WEBOTS_DIR}/lib/controller)
if (WIN32)
  set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX})
else()
  set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

# Include directories
include_directories(${WEBOTS_DIR}/include/controller/c ${WEBOTS_DIR}/include/controller/cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ONNXRUNTIME_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES} ${CONTROLLER_SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES} ${OpenCV_LIBS} ${ONNXRUNTIME_LIBRARIES} slam_pipeline)
add_dependencies (${PROJECT} slam_pipeline)

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} 
                     POST_BUILD 
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     $<TARGET_FILE:${PROJECT}> 
                     ${CMAKE_SOURCE_DIR}
)

# Copy OpenCV dlls
if (MSVC)
message("OpenCV install dir: ${OpenCV_INSTALL_PATH}")
file(GLOB OPENCV_DLLS "${OpenCV_INSTALL_PATH}/x64/vc14/bin/*.dll")
add_custom_command(TARGET ${PROJECT}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${OPENCV_DLLS}
                     ${CMAKE_SOURCE_DIR}
 )
endif (MSVC)

# Copy Onnx dlls
if (MSVC)
file(GLOB TORCH_DLLS "${ONNXRUNTIME_LIBRARY_DIRS}/*.dll")
add_custom_command(TARGET ${PROJECT}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     ${CMAKE_SOURCE_DIR}
 )
endif (MSVC)