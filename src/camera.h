//
// Created by li on 22/8/2021.
//

#ifndef XSTRACKING_CAMERA_H
#define XSTRACKING_CAMERA_H
#include "common.h"

class CameraFactory{
public:
    CameraFactory()=default;
    static std::shared_ptr<Camera> CreateCamera(CAMERA_TYPE cam_type, std::string path);
};

class UVC: public Camera{
public:
    UVC(int cam);
    UVC(std::string path);
    bool GetImg(Ximg &img);

protected:
    cv::VideoCapture fb_;
};

class XSCAM: public Camera{
public:
    XSCAM(std::string path);
    bool GetImg(Ximg &img);

protected:
};




#endif //XSTRACKING_CAMERA_H
