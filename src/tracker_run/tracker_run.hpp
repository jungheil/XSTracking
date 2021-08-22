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

#include "src/common.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include "cf_tracker.hpp"
#include "tracker_debug.hpp"
#include "dsst_tracker.hpp"
#include "usart.h"

enum UsartStatus{
    UsartStatusBringup = 0x01,
    UsartStatusFree = 0x02,
    UsartStatusInit = 0x03,
    UsartStatusTracking = 0x04,
    UsartStatusLoss = 0x05,
    UsartStatusError = 0x06
};

struct UsartSend{
    UsartStatus status_;
    double pitch_;
    double yaw_;
    uint8_t error = 0x00;
    XSTime time_;
};

enum UsartCommand{
    UsartCommandFree = 0x30,
    UsartCommandTarInit = 0x31,
    UsartCommandTarInitAt = 0x32,
    UsartCommandStopTrack = 0x33,
    UsartCommandStopZoom = 0x35
};
enum UsartCamera{
    UsartCameraRGB = 0x01,
    UsartCameraIR = 0x02
};

struct UsartRecv{
    UsartCommand command_;
    UsartCamera camera_;
    uint16_t x_;
    uint16_t y_;
    uint16_t width_;
    uint16_t height_;
    uint16_t zoom_x_;
    uint16_t zoom_y_;
    uint16_t zoom_width_;
    uint16_t zoom_height_;
    XSTime time_;
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
    explicit TrackerRun(std::string windowTitle);
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

    void UsartThread();
    void CameraThread(std::shared_ptr<Camera> cam);
    void CameraThreadFactory(std::vector<std::shared_ptr<Camera>> cams);
    void TrackingThread();

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

    std::vector<std::shared_ptr<Camera>> cameras_;
    std::vector<std::thread> thread_pool_;
    std::atomic<bool> exit_;
    // 临界资源
    UsartSend usart_send_ = {};
    UsartRecv usart_recv_ = {};
    std::mutex usart_send_mutex_;
    std::mutex usart_recv_mutex_;

};


#endif
