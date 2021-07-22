//
// Created by super on 18-7-16.
//
/*
// Original file: https://github.com/Itseez/opencv_contrib/blob/292b8fa6aa403fb7ad6d2afadf4484e39d8ca2f1/modules/tracking/samples/tracker.cpp
// * Refactor file: Move target selection to separate class/file
// * Replace command line argumnets
// * Change tracker calling code
// * Add a variety of additional features
*/

#ifndef TRACKER_RUN_HPP_
#define TRACKER_RUN_HPP_

#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "cf_tracker.hpp"
#include "tracker_debug.hpp"
#include "dsst_tracker.hpp"
#include "usart.h"


struct UsartSend{
    bool isTracking;
    double pitch;
    double yaw;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
};
struct UsartRecv{
    bool update;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
};
class XSUsart:public Usart{
public:
    XSUsart():Usart(){};
    XSUsart(char* device, uint32_t baudrate): Usart(device,baudrate){};
    bool Send(void *msg) override;
    bool Recv(void *msg) override;
};

struct Parameters{
    std::string outputFilePath;
    std::string imgExportPath;
    std::string videoPath;
    std::string usartDevice;
    int usartBaudrate;
    int device;
    int deviceId;
    bool showOutput;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
};

class TrackerRun
{
public:
    TrackerRun(std::string windowTitle);
    virtual ~TrackerRun();
    bool start();
    void setTrackerDebug(cf_tracking::TrackerDebug* debug);

private:
    Parameters parseCmdArgs();
    bool init();
    bool run();
    bool update();
    void printResults(const cv::Rect_<double>& boundingBox, bool isConfident, double fps);
    // TODO: 目前忽略相机到关节的转换
    void AngleResolve(const cv::Rect_<double>& boundingBox, double &pitch, double &yaw);
    bool SendMsg(bool isTracking,cv::Rect_<double> box);
protected:
    virtual cf_tracking::CfTracker* parseTrackerParas(bool enableDebug=false) = 0;
    std::string _trackerConfigPath = "../config/trackerConfig.yaml";
    std::string _runConfigPath = "../config/runConfig.yaml";

private:
    cv::Mat _image;
    cv::Size _imgSize;
    cf_tracking::CfTracker* _tracker;
    std::string _windowTitle;
    Parameters _paras;
    cv::Rect_<double> _boundingBox;
    cv::VideoCapture _cap;
    std::ofstream _resultsFile;
    cf_tracking::TrackerDebug* _debug;
    XSUsart _usart;
    int _frameIdx;
    bool _isPaused = false;
    bool _isStep = false;
    bool _exit = false;
    bool _hasInitBox = false;
    bool _isTrackerInitialzed = false;
    bool _targetOnFrame = false;
    bool _updateAtPos = false;
};


#endif
