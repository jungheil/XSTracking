//
// Created by super on 18-7-16.
//
/*
// Original file: https://github.com/Itseez/opencv_contrib/blob/292b8fa6aa403fb7ad6d2afadf4484e39d8ca2f1/modules/tracking/samples/tracker.cpp
// * Replace command line arguments
// * Add a variety of additional features
*/

#include "tracker_run.hpp"
#include<iomanip>


#include <iostream>
#include <ctype.h>
#include <math.h>

#include <thread>

#include "init_box_selector.hpp"
#include "cf_tracker.hpp"
#include "src/camera.h"

using namespace cv;
using namespace std;
using namespace cf_tracking;

TrackerRun::TrackerRun(string windowTitle) :
_windowTitle(windowTitle),
_debug(0),
exit_(false)
{
    _tracker = 0;
}

TrackerRun::~TrackerRun()
{
    if (_resultsFile.is_open())
        _resultsFile.close();

    if (_tracker)
    {
        delete _tracker;
        _tracker = 0;
    }
}

Parameters TrackerRun::parseCmdArgs()
{
    Parameters paras;
    bool enableDebug=false;

    cv::FileStorage fs(_runConfigPath , cv::FileStorage::READ);
    if(!fs.isOpened()){
        cerr<<"load param failed!"<<endl;
        exit(100);
    }
    if(!fs["cameraType"].empty()) {
        cv::FileNode deviceNode = fs["cameraType"];
        auto it=deviceNode.begin(), it_end=deviceNode.end();
        for(;it!=it_end;++it){
            paras.device.push_back((int)(*it));
        }
    }else cerr << "missing parameters!" << endl;
    if(!fs["deviceId"].empty()){
        cv::FileNode deviceIdNode = fs["deviceId"];
        auto it=deviceIdNode.begin(), it_end=deviceIdNode.end();
        for(;it!=it_end;++it){
            paras.deviceId.push_back((int)(*it));
        }
    }else cerr << "missing parameters!" << endl;
    if(!fs["videoPath"].empty()){
        cv::FileNode videoPathNode = fs["videoPath"];
        auto it=videoPathNode.begin(), it_end=videoPathNode.end();
        for(;it!=it_end;++it){
            paras.videoPath.push_back((std::string)(*it));
        }
    }else cerr << "missing parameters!" << endl;

    if(!fs["usartDevice"].empty()) fs["usartDevice"]>>paras.usartDevice;
    else cerr << "missing parameters!" << endl;
    if(!fs["usartBaudrate"].empty()) fs["usartBaudrate"]>>paras.usartBaudrate;
    else cerr << "missing parameters!" << endl;

    if(!fs["outputFilePath"].empty()) fs["outputFilePath"]>>paras.outputFilePath;
    else cerr << "missing parameters!" << endl;
    if(!fs["imgExportPath"].empty()) fs["imgExportPath"]>>paras.imgExportPath;
    else cerr << "missing parameters!" << endl;
    if(!fs["showOutput"].empty()) fs["showOutput"]>>paras.showOutput;
    else cerr << "missing parameters!" << endl;
    if(!fs["localDebug"].empty()) fs["localDebug"]>>paras.localDebug;
    else cerr << "missing parameters!" << endl;
    if(!fs["videoDebug"].empty()) fs["videoDebug"]>>paras.videoDebug;
    else cerr << "missing parameters!" << endl;
    if(!fs["enableDebug"].empty()) fs["enableDebug"]>>enableDebug;
    else cerr << "missing parameters!" << endl;
    if(!fs["timeStamp"].empty()) fs["timeStamp"]>>paras.timeStamp;
    else cerr << "missing parameters!" << endl;

    if(!fs["cameraMatrix"].empty()){
        cv::FileNode cameraMatrixNode = fs["cameraMatrix"];
        auto it=cameraMatrixNode.begin(), it_end=cameraMatrixNode.end();
        for(;it!=it_end;++it){
            cv::Mat t;
            *it >> t;
            paras.cameraMatrix.emplace_back(std::move(t));
        }
    }else cerr << "missing parameters!" << endl;
    if(!fs["distCoeffs"].empty()){
        cv::FileNode distCoeffsNode = fs["distCoeffs"];
        auto it=distCoeffsNode.begin(), it_end=distCoeffsNode.end();
        for(;it!=it_end;++it){
            cv::Mat t;
            *it >> t;
            paras.distCoeffs.emplace_back(std::move(t));
        }
    }else cerr << "missing parameters!" << endl;

    fs.release();

    _tracker = parseTrackerParas(enableDebug);
    char *usartDevice = const_cast<char*>(paras.usartDevice.c_str());
    if(!paras.usartDevice.empty()){
        _usart = XSUsart(usartDevice,paras.usartBaudrate);
    }

    if (enableDebug)
        _debug->init(paras.outputFilePath + "_debug");
    else
        _debug=0;

    return paras;
}

bool TrackerRun::start()
{
    _paras = parseCmdArgs();

    while (true)
    {
        if (!init())
            return false;
        if (!run())
            return false;

        if (exit_)
            break;

        _hasInitBox = false;
        _isTrackerInitialzed = false;
    }

    return true;
}

bool TrackerRun::init()
{
    int cam_count = 0;
    for(auto& dev:_paras.device){
        switch(dev){
            case 0:
                cameras_.emplace_back(
                        CameraFactory::CreateCamera(
                                CAMERA_TYPE_UVC,
                                _paras.deviceId[cam_count++]));
                break;
            case 1:
                cameras_.emplace_back(
                        CameraFactory::CreateCamera(
                                CAMERA_TYPE_UVC,
                                _paras.videoPath[cam_count++]));
                break;
            case 2:
                cameras_.emplace_back(
                        CameraFactory::CreateCamera(
                                CAMERA_TYPE_XSCAM,
                                _paras.videoPath[cam_count++]));
                break;
        }
    }

    for(auto &s:cameras_){
        Ximg temp_img;
        s->GetImg(temp_img);
        _imgSize.emplace_back(cv::Size(temp_img.get_cv_color().cols,temp_img.get_cv_color().rows));
        _image.push_back(std::move(temp_img));
        img_mutex_.emplace_back(make_unique<std::mutex>());
        img_used_.emplace_back(make_unique<std::atomic<bool>>(true));
        img_cache_.emplace_back(make_unique<ImgCache>());
    }

#ifndef TERMINAL_MODE
    if (_paras.showOutput)
        namedWindow(_windowTitle.c_str());
#endif
    if (!_paras.outputFilePath.empty())
    {
        _resultsFile.open(_paras.outputFilePath.c_str());

        if (!_resultsFile.is_open())
        {
            std::cerr << "Error: Unable to create results file: "
                << _paras.outputFilePath.c_str()
                << std::endl;

            return false;
        }

        _resultsFile.precision(std::numeric_limits<double>::digits10 - 4);
    }

    _frameIdx = 0;

    usart_send_mutex_.lock();
    usart_send_.status_=UsartStatusBringup;
    usart_send_mutex_.unlock();
    thread_pool_.emplace_back(std::thread(&TrackerRun::UsartThread,this));
    CameraThreadFactory(cameras_);
    //TODO 解决线程启动顺序问题
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return true;
}

bool TrackerRun::run()
{
    std::cout << "Switch pause with 'p'" << std::endl;
    std::cout << "Step frame with 'c'" << std::endl;
    std::cout << "Select new target with 'r'" << std::endl;
    std::cout << "Update current tracker model at new location  't'" << std::endl;
    std::cout << "Quit with 'ESC'" << std::endl;

    thread_pool_.emplace_back(std::thread(&TrackerRun::TrackingThread,this));
    // TODO: 主线程要空闲吗？
    while(!exit_){}

    for(auto &t:thread_pool_){
        t.join();
    }

    return true;
}

bool TrackerRun::update()
{
    Ximg src;
    int64 tStart = 0;
    int64 tDuration = 0;
//    bool isEnd = false;

    usart_recv_mutex_.lock();
    recv_ = usart_recv_;
    usart_recv_mutex_.unlock();

    if(!_paras.localDebug){
        switch(recv_.command_){
            case UsartCommandFree:
                _isPaused = true;
                _hasInitBox = false;
                _isTrackerInitialzed = false;
                usart_send_mutex_.lock();
                usart_send_.status_ = UsartStatusFree;
                usart_send_mutex_.unlock();
                return true;
                break;
            case UsartCommandTarInit:
                if(tracking_tar_==recv_.tar_id_){
                    _isPaused = false;
                }else{
                    _hasInitBox = false;
                    _isTrackerInitialzed = false;
                }
                break;
            case UsartCommandTarInitAt:
                if(tracking_tar_==recv_.tar_id_) {
                    _isPaused = false;
                }else{
                    _updateAtPos = true;
                }
                break;
            case UsartCommandStopTrack:
                _isPaused = true;
                break;
            case UsartComandStartTrack:
                _isPaused = false;
                break;
            default:
                std::cerr<<"Usart command error"<<endl;
                return true;
        }
    }

    if (!_isPaused || _frameIdx == 0 || _isStep)
    {
        if(_paras.localDebug){

            while(*img_used_[0]){}

            img_mutex_[0]->lock();
            src = _image[0];
            img_mutex_[0]->unlock();
            *img_used_[0]=true;
        }else{
            if(_paras.timeStamp){
                if(recv_.camera_ == 0x01){
                    while(*img_used_[0]){}
                    img_mutex_[0]->lock();
                    img_cache_[0]->Get(recv_.time_,src);
                    img_mutex_[0]->lock();
                    *img_used_[0]=true;
                }else if(recv_.camera_ == 0x02){
                    while(*img_used_[1]){}
                    img_mutex_[1]->lock();
                    img_cache_[1]->Get(recv_.time_,src);
                    img_mutex_[1]->lock();
                    *img_used_[1]=true;
                }else{
                    std::cerr<<"camera type error"<<endl;
                    return true;
                }
            }else{
                if(recv_.camera_ == 0x01){
                    while(*img_used_[0]){}
                    img_mutex_[0]->lock();
                    src = _image[0];
                    img_mutex_[0]->lock();
                    *img_used_[0]=true;
                }else if(recv_.camera_ == 0x02){
                    while(*img_used_[1]){}
                    img_mutex_[1]->lock();
                    src = _image[1];
                    img_mutex_[1]->lock();
                    *img_used_[1]=true;
                }else{
                    std::cerr<<"camera type error"<<endl;
                    return true;
                }
            }
        }
        if (src.get_cv_color().empty())
            return false;

        usart_send_mutex_.lock();
        usart_send_.time_=src.get_time();
        usart_send_mutex_.unlock();

        ++_frameIdx;
    }

    if (!_isTrackerInitialzed)
    {
        if (!_hasInitBox)
        {
            usart_send_mutex_.lock();
            usart_send_.status_ = UsartStatusInit;
            usart_send_mutex_.unlock();
            Rect box;
            if(_paras.localDebug){
#ifndef TERMINAL_MODE
                if (!InitBoxSelector::selectBox(src.get_cv_color(), box))
                    return false;
#endif
            }else{
                float zoom_s = recv_.zoom_size_/src.get_cv_color().cols;
                box = Rect(int(recv_.zoom_x_+recv_.x_*zoom_s),
                           int(recv_.zoom_y_+recv_.y_*zoom_s),
                           int(recv_.width_*zoom_s),
                           int(recv_.height_*zoom_s));
            }
            _boundingBox = Rect_<double>(static_cast<double>(box.x),
                static_cast<double>(box.y),
                static_cast<double>(box.width),
                static_cast<double>(box.height));

            _hasInitBox = true;
            tracking_tar_=recv_.tar_id_;
        }

        tStart = getTickCount();
        _targetOnFrame = _tracker->reinit(src.get_cv_color(), _boundingBox);
        tDuration = getTickCount() - tStart;

        if (_targetOnFrame)
            _isTrackerInitialzed = true;
    }
    else if (_isTrackerInitialzed && (!_isPaused || _isStep))
    {

        _isStep = false;

        if (_updateAtPos)
        {
            usart_send_mutex_.lock();
            usart_send_.status_ = UsartStatusInit;
            usart_send_mutex_.unlock();
            Rect box;

            if(_paras.localDebug){
#ifndef TERMINAL_MODE

                if (!InitBoxSelector::selectBox(src.get_cv_color(), box))
                    return false;
#endif
            }else{
                float zoom_s = recv_.zoom_size_/src.get_cv_color().cols;
                box = Rect(int(recv_.zoom_x_+recv_.x_*zoom_s),
                           int(recv_.zoom_y_+recv_.y_*zoom_s),
                           int(recv_.width_*zoom_s),
                           int(recv_.height_*zoom_s));
            }

            _boundingBox = Rect_<double>(static_cast<double>(box.x),
                static_cast<double>(box.y),
                static_cast<double>(box.width),
                static_cast<double>(box.height));

            _updateAtPos = false;

            std::cout << "UpdateAt_: " << _boundingBox << std::endl;
            tStart = getTickCount();
            _targetOnFrame = _tracker->updateAt(src.get_cv_color(), _boundingBox);
            tDuration = getTickCount() - tStart;

            if (!_targetOnFrame)
                std::cout << "Target not found!" << std::endl;

            tracking_tar_=recv_.tar_id_;
        }
        else
        {
            usart_send_mutex_.lock();
            usart_send_.status_ = UsartStatusTracking;
            usart_send_mutex_.unlock();

            tStart = getTickCount();
            _targetOnFrame = _tracker->update(src.get_cv_color(), _boundingBox);
            tDuration = getTickCount() - tStart;
        }
    }

    if (!_targetOnFrame)
    {
        usart_send_mutex_.lock();
        usart_send_.status_ = UsartStatusLoss;
        usart_send_mutex_.unlock();
    }

    double fps = static_cast<double>(getTickFrequency() / tDuration);
    printResults(_boundingBox, _targetOnFrame, fps);

#ifndef TERMINAL_MODE
    if (_paras.showOutput)
    {
        Mat hudImage;
        src.get_cv_color().copyTo(hudImage);
        rectangle(hudImage, _boundingBox, Scalar(0, 0, 255), 2);
        Point_<double> center;
        center.x = _boundingBox.x + _boundingBox.width / 2;
        center.y = _boundingBox.y + _boundingBox.height / 2;
        circle(hudImage, center, 3, Scalar(0, 0, 255), 2);

        stringstream ss;
        ss << "FPS: " << fps;
        putText(hudImage, ss.str(), Point(20, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0));

        ss.str("");
        ss.clear();
        ss << "#" << _frameIdx;
        putText(hudImage, ss.str(), Point(hudImage.cols - 60, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0));

        if (_debug != 0)
            _debug->printOnImage(hudImage);

        if (!_targetOnFrame)
        {
            cv::Point_<double> tl = _boundingBox.tl();
            cv::Point_<double> br = _boundingBox.br();

            line(hudImage, tl, br, Scalar(0, 0, 255));
            line(hudImage, cv::Point_<double>(tl.x, br.y),
                cv::Point_<double>(br.x, tl.y), Scalar(0, 0, 255));
        }
        imshow(_windowTitle.c_str(), hudImage);

        if (!_paras.imgExportPath.empty())
        {
            stringstream ssi;
            ssi << setfill('0') << setw(5) << _frameIdx << ".png";
            std::string imgPath = _paras.imgExportPath + ssi.str();

            try
            {
                imwrite(imgPath, hudImage);
            }
            catch (runtime_error& runtimeError)
            {
                cerr << "Could not write output images: " << runtimeError.what() << endl;
            }
        }


        char c = (char)waitKey(10);

        if (c == 27)
        {
            exit_ = true;
            return false;
        }

        switch (c)
        {
        case 'p':
            _isPaused = !_isPaused;
            break;
        case 'c':
            _isStep = true;
            break;
        case 'r':
            _hasInitBox = false;
            _isTrackerInitialzed = false;
            break;
        case 't':
            _updateAtPos = true;
            break;
        default:
            ;
        }
    }
#endif


    return true;
}

void TrackerRun::printResults(const cv::Rect_<double>& boundingBox, bool isConfident, double fps)
{
    if (_resultsFile.is_open())
    {
        if (boundingBox.width > 0 && boundingBox.height > 0 && isConfident)
        {
            _resultsFile << boundingBox.x << ","
                << boundingBox.y << ","
                << boundingBox.width << ","
                << boundingBox.height << ","
                << fps << std::endl;
        }
        else
        {
            _resultsFile << "NaN, NaN, NaN, NaN, " << fps << std::endl;
        }

        if (_debug != 0)
            _debug->printToFile();
    }
}

void TrackerRun::setTrackerDebug(cf_tracking::TrackerDebug* debug)
{
    _debug = debug;
}

void TrackerRun::AngleResolve(const Rect_<double> &boundingBox, double &pitch, double &yaw, int cam_id) {
    double x = boundingBox.width/2 + boundingBox.x;
    double y = boundingBox.height/2 + boundingBox.y;

    // TODO: 忽略畸变参数
    pitch = atan2((_paras.cameraMatrix[cam_id].at<double>(1,2)-y),
            _paras.cameraMatrix[cam_id].at<double>(1,1));
    yaw = atan2((_paras.cameraMatrix[cam_id].at<double>(0,2)-x),
                  _paras.cameraMatrix[cam_id].at<double>(0,0));
}

void TrackerRun::UsartThread() {
    UsartSend send_msg;
    UsartRecv recv_msg;
    while(!exit_){
        cout<<"usart thread started!"<<endl;

        usart_send_mutex_.lock();
        send_msg=usart_send_;
        usart_send_mutex_.unlock();
        _usart.Send((void*)&send_msg);


        _usart.Recv((void*)&recv_msg);
        usart_recv_mutex_.lock();
        usart_recv_=recv_msg;
        usart_recv_mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void TrackerRun::CameraThreadFactory(std::vector<std::shared_ptr<Camera>> cams) {
    for(auto &p:cams){
        thread_pool_.emplace_back(&TrackerRun::CameraThread,this,p);
    }
}

void TrackerRun::CameraThread(std::shared_ptr<Camera> cam) {

    cout<<"cam thread started! "<<(int)cam->get_id()<<endl;
    while(!exit_){
        if(!*img_used_[cam->get_id()]) continue;
        Ximg temp_img;
        cam->GetImg(temp_img);
        img_mutex_[cam->get_id()]->lock();
        _image[cam->get_id()] = std::move(temp_img);
        *img_used_[cam->get_id()]=false;
        img_cache_[cam->get_id()]->PushImg(_image[cam->get_id()]);
        img_mutex_[cam->get_id()]->unlock();
    }
}

void TrackerRun::TrackingThread() {
    cout<<"tracking thread started!"<<endl;

    bool success = true;
    usart_send_mutex_.lock();
    usart_send_.status_ = UsartStatusFree;
    usart_send_mutex_.unlock();

    while (true)
    {
        success = update();
        if (!success){
            exit_ = true;
            break;
        }
        usart_send_mutex_.lock();
        AngleResolve(_boundingBox,usart_send_.pitch_,usart_send_.yaw_,recv_.camera_);
        usart_send_mutex_.unlock();
    }
}

//bool TrackerRun::SendMsg(bool isTracking, cv::Rect_<double> box) {
//    UsartSend msg;
//    msg.isTracking = isTracking;
//    double pitch,yaw;
//    AngleResolve(box,pitch,yaw);
//    msg.pitch=pitch;
//    msg.yaw = yaw;
//    msg.x = box.x;
//    msg.y = box.y;
//    msg.width = box.width;
//    msg.height = box.height;
//    return _usart.Send((void*)&msg);
//}

bool XSUsart::Send(void *msg) {
    uint8_t buff[25];
    memset(buff,0,25);
    auto *p = (UsartSend*)msg;
    double yaw = p->yaw_*(180/3.1415926)*100;
    double pitch = p->pitch_*(180/3.1415926)*100;

    buff[0] = 0xEB;
    buff[1] = 0x90;
    buff[2] = p->status_;
    buff[3] = (uint16_t)pitch & 0x00FF;
    buff[4] = (uint16_t)pitch >>8;
    buff[5] = (uint16_t)yaw & 0x00FF;
    buff[6] = (uint16_t)yaw >>8;
    buff[9] = (p->time_).msec_ & 0x00FF;
    buff[10] = (p->time_).msec_ >>8;
    buff[11] = (p->time_).sec_;
    buff[12] = (p->time_).min_;
    buff[13] = (p->time_).hour_;
    buff[14] = (p->time_).day_;
    buff[19] = p->error;


    if(Write(buff,25)){
        return true;
    }else{
        return false;
    }
}

bool XSUsart::Recv(void *msg) {
    uint8_t buff[35];
    bool ret = Read(buff,35);
    if(!ret || (buff[0]!=0xEB && buff[1]!=0x90)){
        return false;
    }
    auto *p = (UsartRecv*)msg;

    p->command_ = (UsartCommand)buff[2];
    p->x_ = (uint16_t)(buff[3] | buff[4]<<8);
    p->y_ = (uint16_t)(buff[5] | buff[6]<<8);
    p->width_ = (uint16_t)(buff[7] | buff[8]<<8);
    p->height_ = (uint16_t)(buff[9] | buff[10]<<8);
    (p->time_).msec_ = (uint16_t)(buff[11] | buff[12]<<8);
    (p->time_).sec_ = (uint8_t)buff[13];
    (p->time_).min_ = (uint8_t)buff[14];
    (p->time_).hour_ = (uint8_t)buff[15];
    (p->time_).day_ = (uint8_t)buff[16];
    p->tar_id_ = (uint8_t)buff[17];
    p->camera_ = (UsartCamera)buff[21];
    p->zoom_size_ = (uint16_t)buff[22];
    p->zoom_x_ = (uint16_t)(buff[23] | buff[24]<<8);
    p->zoom_y_ = (uint16_t)(buff[25] | buff[26]<<8);
}
