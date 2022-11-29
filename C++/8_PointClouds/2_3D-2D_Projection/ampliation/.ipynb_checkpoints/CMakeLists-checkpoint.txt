cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(MYPROJECT)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (camera_calibration camera_calibration.cpp)
target_link_libraries (camera_calibration ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_executable (projection projection.cpp)
target_link_libraries (projection ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_executable (projectionSerpiente projectionSerpiente.cpp)
target_link_libraries (projectionSerpiente ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

