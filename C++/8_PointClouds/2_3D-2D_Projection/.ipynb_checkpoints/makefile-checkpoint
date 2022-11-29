CXX = g++
CXXFLAGS = --std=c++14 -o $@
OPENCV = `pkg-config opencv --cflags --libs`

all: projection camera_calibration
	

projection: projection.cpp
	${CXX} ${CXXFLAGS} $^ ${OPENCV}

camera_calibration: camera_calibration.cpp
	${CXX} ${CXXFLAGS} $^ ${OPENCV}


