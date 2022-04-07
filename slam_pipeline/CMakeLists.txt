cmake_minimum_required(VERSION 3.20)
set(SLAM_PROJECT_NAME slam_pipeline)

option(SLAM_SHARED_LIBS "Build statuc or shared lib" OFF)

if (SLAM_SHARED_LIBS)
  set(LIB_TYPE SHARED)
else()
  set(LIB_TYPE STATIC)
endif()

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(G2O REQUIRED)

SET(G2O_LIBS  
  debug ${G2O_STUFF_LIBRARY_DEBUG} optimized ${G2O_STUFF_LIBRARY}
  debug ${G2O_CORE_LIBRARY_DEBUG} optimized ${G2O_CORE_LIBRARY}
  debug ${G2O_SOLVER_DENSE_DEBUG} optimized ${G2O_SOLVER_DENSE}
  debug ${G2O_SOLVER_EIGEN_DEBUG} optimized ${G2O_SOLVER_EIGEN}
  debug ${G2O_TYPES_SLAM3D_DEBUG} optimized ${G2O_TYPES_SLAM3D}
  debug ${G2O_TYPES_DATA_DEBUG} optimized ${G2O_TYPES_DATA}
  debug ${G2O_TYPES_SIM3_DEBUG} optimized ${G2O_TYPES_SIM3}
  debug ${G2O_TYPES_SBA_DEBUG} optimized ${G2O_TYPES_SBA})

set(LIBS_SLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
${G2O_LIBS}
)

include_directories(
${CMAKE_CURRENT_BINARY_DIR}
${CMAKE_CURRENT_SOURCE_DIR}/
${CMAKE_CURRENT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${G2O_INCLUDE_DIR}
)

set(INCLUDES 
include/System.h
include/Tracking.h
include/LocalMapping.h
# include/LoopClosing.h
include/FeatureMatcher.h
include/MapDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/KeyPointMap.h
include/Map.h
include/Optimizer.h
include/PnPsolver.h
include/FrameBase.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Initializer.h
)


set(SOURCES 
src/FeatureMatcher.cc
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
# src/LoopClosing.cc
src/MapDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/KeyPointMap.cc
src/Map.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/FrameBase.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
)


add_library(${SLAM_PROJECT_NAME} ${LIB_TYPE} ${SOURCES} ${INCLUDES})

# this was already done
# include(GenerateExportHeader)
# generate_export_header(${SLAM_PROJECT_NAME})
if (NOT SLAM_SHARED_LIBS)
  target_compile_definitions(${SLAM_PROJECT_NAME} PUBLIC SLAM_PIPELINE_STATIC_DEFINE)
endif()

target_compile_definitions(${SLAM_PROJECT_NAME} PUBLIC G2O_USE_VENDORED_CERES G2O_SHARED_LIBS)

if (MSVC)
  target_compile_options(${SLAM_PROJECT_NAME} PUBLIC /bigobj)
else ()
  target_compile_options(${SLAM_PROJECT_NAME} PUBLIC -Wa,-mbig-obj)
endif ()

target_link_libraries(${SLAM_PROJECT_NAME} ${LIBS_SLAM})
