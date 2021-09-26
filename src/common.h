//
// Created by li on 20/8/2021.
//

#ifndef XSTRACKING_COMMON_H
#define XSTRACKING_COMMON_H
#include <stdint.h>
#include <iostream>
#include <opencv2/core.hpp>

//#include <opencv2/opencv.hpp>
typedef struct XSTime{
    uint8_t day_;
    uint8_t hour_;
    uint8_t min_;
    uint8_t sec_;
    uint16_t msec_;
} XSTime;

class Camera;

class Ximg{
public:
    Ximg() = default;

    Ximg(Camera *cam, cv::Mat&& src, uint64_t seq=0):
    cam_(cam),
    img_(std::move(src)),
    seq_(seq){};

    Ximg(Camera *cam, cv::Mat&& src, XSTime time, uint64_t seq=0):
    cam_(cam),
    img_(std::move(src)),
    time_(time),
    seq_(seq){};

    inline cv::Mat& get_cv_color() {return img_;};
    inline const XSTime& get_time() {return time_;};
    inline uint64_t get_seq() const {return seq_;};
    inline const Camera* get_cam() {return cam_;};

private:
    Camera const *cam_;
    cv::Mat img_;
    XSTime time_={};
    uint64_t seq_ = 0;
};
enum CAMERA_TYPE{
    CAMERA_TYPE_UNKOWN = 0,
    CAMERA_TYPE_XSCAM = 1,
    CAMERA_TYPE_UVC = 2
};
class Camera {
public:
    Camera() { std::cout<<(int)cam_count_<<std::endl; };
    virtual bool GetImg(Ximg &img)=0;
    bool operator >>(Ximg &img){return GetImg(img);};
    inline CAMERA_TYPE GetCamType(){return cam_type_;};
    inline uint8_t get_id() const {return cam_id_;};
protected:
    CAMERA_TYPE cam_type_ = CAMERA_TYPE_UNKOWN;
    uint8_t cam_id_=255;
    static uint8_t cam_count_;
};


#endif //XSTRACKING_COMMON_H
