//
// Created by li on 22/8/2021.
//
#include "camera.h"


UVC::UVC(int cam) {
    cam_type_ = CAMERA_TYPE_UVC;
    fb_ = cv::VideoCapture(cam);
    if(fb_.isOpened()){
        cam_id_=cam_count_++;
    }
}

UVC::UVC(std::string path) {
    cam_type_ = CAMERA_TYPE_UVC;
    fb_ = cv::VideoCapture(path);
    if(fb_.isOpened()){
        cam_id_=cam_count_++;
    }
}

bool UVC::GetImg(Ximg &img) {
    cv::Mat src;
    fb_ >> src;
    img = Ximg(this,std::move(src));
    return true;
}

std::shared_ptr<Camera> CameraFactory::CreateCamera(CAMERA_TYPE cam_type, std::string path) {
    switch(cam_type){
        case CAMERA_TYPE_UVC:
            return std::make_shared<UVC>(path);
            break;
        case CAMERA_TYPE_XSCAM:
            return std::make_shared<XSCAM>(path);
            break;
        default:
            std::cerr<<"camera type invalid!"<<std::endl;
    }
}

std::shared_ptr<Camera> CameraFactory::CreateCamera(CAMERA_TYPE cam_type, int id) {
    switch(cam_type){
        case CAMERA_TYPE_UVC:
            return std::make_shared<UVC>(id);
            break;
        default:
            std::cerr<<"camera type invalid!"<<std::endl;
    }
}

XSCAM::XSCAM(std::string path) {

}

bool XSCAM::GetImg(Ximg &img) {

}
