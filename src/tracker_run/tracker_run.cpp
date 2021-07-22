//
// Created by super on 18-7-16.
//
/*
// Original file: https://github.com/Itseez/opencv_contrib/blob/292b8fa6aa403fb7ad6d2afadf4484e39d8ca2f1/modules/tracking/samples/tracker.cpp
// * Replace command line arguments
// * Add a variety of additional features
*/

#include "tracker_run.hpp"

#include <iostream>
#include <ctype.h>
#include <math.h>

#include "init_box_selector.hpp"
#include "cf_tracker.hpp"

using namespace cv;
using namespace std;
using namespace cf_tracking;

TrackerRun::TrackerRun(string windowTitle) :
_windowTitle(windowTitle),
_debug(0)
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
    if(!fs["cameraType"].empty()) fs["cameraType"]>>paras.device;
    else cerr << "missing parameters!" << endl;
    if(!fs["deviceId"].empty()) fs["deviceId"]>>paras.deviceId;
    else cerr << "missing parameters!" << endl;
    if(!fs["videoPath"].empty()) fs["videoPath"]>>paras.videoPath;
    else cerr << "missing parameters!" << endl;

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
    if(!fs["enableDebug"].empty()) fs["enableDebug"]>>enableDebug;
    else cerr << "missing parameters!" << endl;

    if(!fs["cameraMatrix"].empty()) fs["cameraMatrix"]>>paras.cameraMatrix;
    else cerr << "missing parameters!" << endl;
    if(!fs["distCoeffs"].empty()) fs["distCoeffs"]>>paras.distCoeffs;
    else cerr << "missing parameters!" << endl;

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
        if (init() == false)
            return false;
        if (run() == false)
            return false;

        if (_exit)
            break;

        _hasInitBox = false;
        _isTrackerInitialzed = false;
    }

    return true;
}

bool TrackerRun::init()
{
    switch(_paras.device){
        case 0:
            _cap.open(_paras.videoPath);
            break;
        case 1:
            _cap.open(_paras.deviceId);
            break;
    }

    if (!_cap.isOpened())
    {
        cerr << "Could not open device/video!" << endl;
        exit(-1);
    }
    _cap >> _image;
    _imgSize = cv::Size(_image.cols,_image.rows);

    if (_paras.showOutput)
        namedWindow(_windowTitle.c_str());

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
    return true;
}

bool TrackerRun::run()
{
    bool success = true;

    std::cout << "Switch pause with 'p'" << std::endl;
    std::cout << "Step frame with 'c'" << std::endl;
    std::cout << "Select new target with 'r'" << std::endl;
    std::cout << "Update current tracker model at new location  't'" << std::endl;
    std::cout << "Quit with 'ESC'" << std::endl;

    while (true)
    {
        success = update();
        SendMsg(_targetOnFrame,_boundingBox);
        if (!success)
            break;
    }

    _cap.release();

    return true;
}

bool TrackerRun::update()
{
    int64 tStart = 0;
    int64 tDuration = 0;
//    bool isEnd = false;

    if (!_isPaused || _frameIdx == 0 || _isStep)
    {
        _cap >> _image;

        if (_image.empty())
            return false;

        ++_frameIdx;
    }

    if (!_isTrackerInitialzed)
    {
        if (!_hasInitBox)
        {
            Rect box;

            if (!InitBoxSelector::selectBox(_image, box))
                return false;

            _boundingBox = Rect_<double>(static_cast<double>(box.x),
                static_cast<double>(box.y),
                static_cast<double>(box.width),
                static_cast<double>(box.height));

            _hasInitBox = true;
        }

        tStart = getTickCount();
        _targetOnFrame = _tracker->reinit(_image, _boundingBox);
        tDuration = getTickCount() - tStart;

        if (_targetOnFrame)
            _isTrackerInitialzed = true;
    }
    else if (_isTrackerInitialzed && (!_isPaused || _isStep))
    {
        _isStep = false;

        if (_updateAtPos)
        {
            Rect box;

            if (!InitBoxSelector::selectBox(_image, box))
                return false;

            _boundingBox = Rect_<double>(static_cast<double>(box.x),
                static_cast<double>(box.y),
                static_cast<double>(box.width),
                static_cast<double>(box.height));

            _updateAtPos = false;

            std::cout << "UpdateAt_: " << _boundingBox << std::endl;
            tStart = getTickCount();
            _targetOnFrame = _tracker->updateAt(_image, _boundingBox);
            tDuration = getTickCount() - tStart;

            if (!_targetOnFrame)
                std::cout << "Target not found!" << std::endl;
        }
        else
        {
            tStart = getTickCount();
            _targetOnFrame = _tracker->update(_image, _boundingBox);
            tDuration = getTickCount() - tStart;
        }
    }

    double fps = static_cast<double>(getTickFrequency() / tDuration);
    printResults(_boundingBox, _targetOnFrame, fps);

    if (_paras.showOutput)
    {
        Mat hudImage;
        _image.copyTo(hudImage);
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
            _exit = true;
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

void TrackerRun::AngleResolve(const Rect_<double> &boundingBox, double &pitch, double &yaw) {
    double x = boundingBox.width/2 + boundingBox.x;
    double y = boundingBox.height/2 + boundingBox.y;

    // TODO: 忽略畸变参数
    pitch = atan2((_paras.cameraMatrix.at<double>(1,2)-y),
            _paras.cameraMatrix.at<double>(1,1));
    yaw = atan2((_paras.cameraMatrix.at<double>(0,2)-x),
                  _paras.cameraMatrix.at<double>(0,0));
}

bool TrackerRun::SendMsg(bool isTracking, cv::Rect_<double> box) {
    UsartSend msg;
    msg.isTracking = isTracking;
    double pitch,yaw;
    AngleResolve(box,pitch,yaw);
    msg.pitch=pitch;
    msg.yaw = yaw;
    msg.x = box.x;
    msg.y = box.y;
    msg.width = box.width;
    msg.height = box.height;
    return _usart.Send((void*)&msg);
}

bool XSUsart::Send(void *msg) {
    uint8_t buff[6];
    memset(buff,0,6);
    auto *p = (UsartSend*)msg;
    double yaw = p->yaw*(180/3.1415926)*100;
    double pitch = p->pitch*(180/3.1415926)*100;

    buff[0] = 0xFF;
    buff[1] |= p->isTracking&0x01;
    buff[2] = (uint16_t)pitch & 0x00FF;
    buff[3] = (uint16_t)pitch >>8;
    buff[4] = (uint16_t)yaw & 0x00FF;
    buff[5] = (uint16_t)yaw >>8;

    if(Write(buff,6)){
        return true;
    }else{
        return false;
    }
}

bool XSUsart::Recv(void *msg) {
    uint8_t buff[10];
    bool ret = Read(buff,10);
    if(!ret || buff[0]!=0xFF){
        return false;
    }
    auto *p = (UsartRecv*)msg;

    p->update = buff[1]&0x01;
    if(!p->update){
        return true;
    }
    p->x = (uint16_t)(buff[2] | buff[3]<<8);
    p->y = (uint16_t)(buff[4] | buff[5]<<8);
    p->width = (uint16_t)(buff[6] | buff[7]<<8);
    p->height = (uint16_t)(buff[8] | buff[9]<<8);
    uint8_t fb = 0xaa;
    return Write(&fb,1);
}