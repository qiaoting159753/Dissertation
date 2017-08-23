cmake_minimum_required(VERSION 3.7)
project(Dissertation)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3.0 QUIET)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        /home/t/Documents/ORB_SLAM2-master/Thirdparty/DBoW2/DBoW2
)

set(SOURCE_FILES main.cpp src/MapPoint.cpp include/MapPoint.h include/Feature.h src/Frame.cpp include/Frame.h src/ORBextractor.cpp include/ORBextractor.h src/ORBmatcher.cpp include/ORBmatcher.h src/Tracking.cpp include/Tracking.h include/Camera.h include/Converter.h include/ORBVocabulary.h src/Map.cpp include/Map.h src/QuasiDenseStereo.cpp include/QuasiDenseStereo.h src/Viewer.cpp include/Viewer.h src/FrameDrawer.cpp include/FrameDrawer.h src/MapDrawer.cpp include/MapDrawer.h)
add_executable(Dissertation ${SOURCE_FILES})

target_link_libraries(
        Dissertation
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${Boost_LIBRARIES}
        /home/t/Documents/ORB_SLAM2-master/Thirdparty/DBoW2/lib/libDBoW2.so
)