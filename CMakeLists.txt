cmake_minimum_required(VERSION 2.8.7)
project(XSTracking)

set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
#add_definitions(-D TERMINAL_MODE)

# add OpenCV
set(OPENCV_DIR_HINT "")

set(OpenCV_STATIC OFF)
find_package(OpenCV REQUIRED HINTS ${OPENCV_DIR_HINT})

# add 3rdparty source
set(CF_TCLAP_DIR "src/3rdparty/tclap")
set(CF_CV_EXT_DIR "src/3rdparty/cv_ext")
set(CF_PIOTR_DIR "src/3rdparty/piotr")
set(CF_TRACKER_RUN "src/tracker_run")

# add 3rdparty source
set(CF_TCLAP_DIR "src/3rdparty/tclap")
set(CF_CV_EXT_DIR "src/3rdparty/cv_ext")
set(CF_PIOTR_DIR "src/3rdparty/piotr")
set(CF_RS232_DIR "src/3rdparty/usart")
set(CF_TRACKER_RUN "src/tracker_run")

include_directories(${OpenCV_DIR}/include)
include_directories(${CF_TCLAP_DIR}
        ${CF_CV_EXT_DIR}
        ${CF_PIOTR_DIR}
        ${CF_PIOTR_DIR}/src
        ${CF_RS232_DIR}
        ${CF_TRACKER_RUN}
        ${PROJECT_SOURCE_DIR})

include_directories(src/cf_libs/kcf
        src/cf_libs/dsst
        src/cf_libs/common
        src/cf_libs/staple)

set(CF_PIOTR_SOURCES
        ${CF_PIOTR_DIR}/gradientMex.hpp
        ${CF_PIOTR_DIR}/src/gradientMex.cpp
        ${CF_PIOTR_DIR}/src/sse.hpp
        ${CF_PIOTR_DIR}/src/wrappers.hpp)

set(CF_LIB_COMMON_SOURCES
        src/cf_libs/common/feature_channels.hpp
        src/cf_libs/common/mat_consts.hpp
        src/cf_libs/common/math_helper.hpp
        src/cf_libs/common/math_helper.cpp
        src/cf_libs/common/cf_tracker.hpp
        src/cf_libs/common/scale_estimator.hpp
        src/cf_libs/common/cv_ext.hpp
        src/cf_libs/common/tracker_debug.hpp
        ${CF_CV_EXT_DIR}/shift.cpp
        ${CF_CV_EXT_DIR}/shift.hpp
        ${CF_CV_EXT_DIR}/math_spectrums.cpp
        ${CF_CV_EXT_DIR}/math_spectrums.hpp
        ${CF_CV_EXT_DIR}/psr.hpp
        ${CF_PIOTR_SOURCES})

set(CF_MAIN_SOURCES
        ${CF_CV_EXT_DIR}/init_box_selector.hpp
        ${CF_CV_EXT_DIR}/init_box_selector.cpp
        ${CF_TRACKER_RUN}/tracker_run.hpp
        ${CF_TRACKER_RUN}/tracker_run.cpp)

add_executable(XSTracking
        src/main.cpp
        src/common.cpp
        src/camera.cpp
        src/cf_libs/dsst/dsst_debug.hpp
        src/cf_libs/dsst/dsst_tracker.hpp
        src/3rdparty/usart/usart.cpp
        ${CF_MAIN_SOURCES}
        ${CF_LIB_COMMON_SOURCES})
#target_link_libraries(XSTracking ${OpenCV_LIBS} -lpthread)
target_link_libraries(XSTracking
        ${OpenCV_LIBS}
        ${AVFORMAT_LIBRARY}
        ${AVDEVICE_LIBRARY}
        ${AVCODEC_LIBRARY}
        ${AVUTIL_LIBRARY}
        -lpthread
        -L/usr/local/lib -lavformat -lm -lbz2 -lz -lavcodec -pthread -lm -llzma -lz -lswresample -lm -lavutil -pthread -lm -lX11
        #在terminal里面输入，然后回车   pkg-config --libs libavutil libavformat libavcodec
        #我的电脑显示的是：
        #-L/usr/local/lib -lavformat -lm -lbz2 -lz -lavcodec -pthread -lm -llzma -lz -lswresample -lm -lavutil -pthread -lm -lX11
        #不同电脑可能不一样，把自己电脑显示的内容代替我这个，就可以了。
        )