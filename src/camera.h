//
// Created by li on 22/8/2021.
//

#ifndef XSTRACKING_CAMERA_H
#define XSTRACKING_CAMERA_H
#include "common.h"
#include <stdint.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <opencv2/videoio.hpp>

extern "C"
{
#include "libavformat/avformat.h"
#include "libavcodec/mediacodec.h"
#include "libavutil/mathematics.h"
#include "libavutil/time.h"
};

class CameraFactory{
public:
    CameraFactory()=default;
    static std::shared_ptr<Camera> CreateCamera(CAMERA_TYPE cam_type, std::string path);
    static std::shared_ptr<Camera> CreateCamera(CAMERA_TYPE cam_type, int id);
};

class UVC: public Camera{
public:
    UVC(int cam);
    UVC(std::string path, bool time_stamp=false);
    bool GetImg(Ximg &img);

protected:
    bool TSInit(std::string path);
    bool GetTS(XSTime &time);
    cv::VideoCapture fb_;
    bool time_stamp_=false;
    int videoindex_;
    AVFormatContext *ifmt_ctx_ = nullptr;
};

class XSCAM: public Camera{
public:
    XSCAM(std::string path);
    bool GetImg(Ximg &img);

protected:
};




#endif //XSTRACKING_CAMERA_H
