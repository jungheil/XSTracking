/* 
Created by li on 22/8/2021.
Modified by Gu on 24/8/2021
 */

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

//    this->path_=path;
//    GetTimestampFromSei(path,UVC_TV);
//    totalTimeNUms = UVC_TV.size();
//    getUVCTimeNums = 0;

}

bool UVC::GetImg(Ximg &img) {
    cv::Mat src;
    fb_ >> src;
    //XSTime Img_time = this->UVC_TV[getUVCTimeNums];
    if(src.channels()==3) //三通道
    {
        img = Ximg(this, std::move(src), TRIPLE, getUVCTimeNums);
    }
    else //单通道
        img = Ximg(this, std::move(src), SINGLE, getUVCTimeNums);
    getUVCTimeNums++;
    //img = Ximg(this,std::move(src));
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
    cam_type_ = CAMERA_TYPE_XSCAM;
    this->path_=path;
    rtspVC = cv::VideoCapture(path);
    GetTimestampFromSei(path,XSCAM_TV);
    totalTimeNUms = XSCAM_TV.size();
    getXSTimeNums = 0;
}

bool XSCAM::GetImg(Ximg &img) {
    cv::Mat src;
    rtspVC >> src;
    XSTime Img_time = this->XSCAM_TV[getXSTimeNums];
    if(src.channels()==3) //三通道
    {
        img = Ximg(this, std::move(src), Img_time, TRIPLE, getXSTimeNums);
    }
    else //单通道
        img = Ximg(this, std::move(src), Img_time, SINGLE, getXSTimeNums);
    
    getXSTimeNums++;
    return true;
}