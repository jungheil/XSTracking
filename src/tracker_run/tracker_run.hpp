#ifndef TRACKER_RUN_HPP_
#define TRACKER_RUN_HPP_

#include <atomic>
#include <fstream>
#include <mutex>
#include <thread>

#include "src/common.h"
#ifndef TERMINAL_MODE
#include <opencv2/highgui/highgui.hpp>
#endif
#include "cf_tracker.hpp"
#include "dsst_tracker.hpp"
#include "tracker_debug.hpp"
#include "usart.h"

enum UsartStatus {
    UsartStatusBringup   = 0x01,
    UsartStatusFree      = 0x02,
    UsartStatusInit      = 0x03,
    UsartStatusTracking  = 0x04,
    UsartStatusLoss      = 0x05,
    UsartStatusError     = 0x06,
    UsartStatusRecording = 0x80
};

struct UsartSend {
    UsartStatus status_;
    double      pitch_;
    double      yaw_;
    uint8_t     error = 0x00;
    XSTime      time_;
    uint16_t    x_;
    uint16_t    y_;
    uint16_t    width_;
    uint16_t    height_;
};

enum UsartCommand {
    UsartCommandFree        = 0x30,
    UsartCommandTarInit     = 0x31,
    UsartCommandTarInitAt   = 0x32,
    UsartCommandStopTrack   = 0x33,
    UsartComandStartTrack   = 0x34,
    UsartCommandStartRecord = 0x35,
    UsartCommandStopRecord  = 0x36
};

enum UsartCamera { UsartCameraRGB = 0x01, UsartCameraIR = 0x02 };

struct UsartRecv {
    UsartCommand command_;
    UsartCamera  camera_;
    uint16_t     x_;
    uint16_t     y_;
    uint16_t     width_;
    uint16_t     height_;
    uint8_t      tar_id_;
    uint16_t     zoom_x_;
    uint16_t     zoom_y_;
    uint8_t      zoom_size_;
    XSTime       time_;
};
class XSUsart : public Usart {
  public:
    XSUsart() : Usart(){};
    XSUsart(char* device, uint32_t baudrate) : Usart(device, baudrate, 37){};
    bool Send(void* msg) override;
    bool Recv(void* msg) override;
};

struct Parameters {
    std::string              outputFilePath;
    std::string              imgExportPath;
    std::vector<std::string> videoPath;
    std::string              usartDevice;
    std::string              savePath;
    int                      usartBaudrate;
    std::vector<int>         device;
    std::vector<int>         deviceId;
    bool                     showOutput;
    bool                     videoDebug;
    bool                     localDebug;
    bool                     timeStamp;
    std::vector<cv::Mat>     cameraMatrix;
    std::vector<cv::Mat>     distCoeffs;
};

class TrackerRun {
  public:
    explicit TrackerRun(std::string windowTitle);
    virtual ~TrackerRun();
    bool start();
    void setTrackerDebug(cf_tracking::TrackerDebug* debug);

  private:
    Parameters parseCmdArgs();
    bool       init();
    bool       run();
    bool       update();
    void printResults(const cv::Rect_<double>& boundingBox, bool isConfident,
                      double fps);
    // TODO: 目前忽略相机到关节的转换
    void AngleResolve(const cv::Rect_<double>& boundingBox, double& pitch,
                      double& yaw, int cam_id);
    bool SendMsg(bool isTracking, cv::Rect_<double> box);

    void UsartThread();
    // TODO: 图像序列，时间
    void CameraThread(std::shared_ptr<Camera> cam);
    void CameraThreadFactory(std::vector<std::shared_ptr<Camera>> cams);
    void TrackingThread();

    [[noreturn]] void RecordThread();

  protected:
    virtual cf_tracking::CfTracker* parseTrackerParas(
        bool enableDebug = false)  = 0;
    std::string _trackerConfigPath = "../config/trackerConfig.yaml";
    std::string _runConfigPath     = "../config/runConfig.yaml";

  private:
    std::vector<cv::Size>   _imgSize;
    cf_tracking::CfTracker* _tracker;
    std::string             _windowTitle;
    Parameters              _paras;
    cv::Rect_<double>       _boundingBox;
    //    cv::VideoCapture _cap;
    std::ofstream              _resultsFile;
    cf_tracking::TrackerDebug* _debug;
    XSUsart                    _usart;
    int                        _frameIdx;
    bool                       _isPaused            = false;
    bool                       _isStep              = false;
    bool                       _hasInitBox          = false;
    bool                       _isTrackerInitialzed = false;
    bool                       _targetOnFrame       = false;
    bool                       _updateAtPos         = false;

    uint8_t tracking_tar_ = 255;

    std::vector<std::shared_ptr<Camera>> cameras_;
    std::vector<std::thread>             thread_pool_;
    std::atomic<bool>                    exit_;
    std::atomic<bool>                    recording;
    UsartSend                            usart_temp_ = {};
    // 临界资源
    UsartSend  usart_send_ = {};
    UsartRecv  usart_recv_ = {};
    std::mutex usart_send_mutex_;
    std::mutex usart_recv_mutex_;

    //    std::vector<Ximg>                               _image;
    std::vector<std::unique_ptr<std::mutex>> img_mutex_;
    //    std::vector<std::unique_ptr<std::atomic<bool>>> img_used_;
    //    std::vector<std::unique_ptr<ImgCache>>          img_cache_;
    std::vector<std::unique_ptr<ImgBuff<20>>>       img_buff_;
    std::vector<std::unique_ptr<std::atomic<bool>>> img_recorded;

    UsartRecv recv_{};
};

#endif
