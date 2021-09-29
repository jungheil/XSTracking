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

UVC::UVC(std::string path, bool time_stamp):time_stamp_(time_stamp) {
    bool ret;
    cam_type_ = CAMERA_TYPE_UVC;
    fb_ = cv::VideoCapture(path);
    ret = fb_.isOpened();
    if (time_stamp_){
        ret &= TSInit(path);
    }
    if(ret){
        cam_id_=cam_count_++;
    }
}

bool UVC::GetImg(Ximg &img) {
    cv::Mat src;
    XSTime time;
    fb_ >> src;
    if(time_stamp_){
        GetTS(time);
        img = Ximg(this,std::move(src),time);
    }else{
        img = Ximg(this,std::move(src));
    }
    return true;
}

bool UVC::TSInit(std::string path){
    int ret, i;

    avformat_network_init(); //初始化网络库

    //avformat_open_input(&ifmt_ctx, in_filename, 0,0)；用来打开文件，解封文件头，若成功，返回0。
    if ((ret = avformat_open_input(&ifmt_ctx_, path.c_str(), 0, 0)) < 0) {
        printf("Could not open input file.");
        return false;
    }

    //avformat_find_stream_info(ifmt_ctx, 0)：用来获取音视频流的信息，若成功，返回0。
    if ((ret = avformat_find_stream_info(ifmt_ctx_, 0)) < 0) {
        printf("Failed to retrieve input stream information");
        return false;
    }

    //作用：寻找视频流。
    for(i=0;i<ifmt_ctx_->nb_streams;i++)
    {
        if(ifmt_ctx_->streams[i]->codecpar->codec_type==AVMEDIA_TYPE_VIDEO)  //如果是视频类型
        {
            videoindex_ = i;
            std::cout<<"the videoindex is :"<<videoindex_<<std::endl;
            break;
        }
    }
    if(videoindex_==-1)
    {
        std::cout<<"can't find a video stream"<<std::endl;
        return false;
    }
}

bool UVC::GetTS(XSTime &time) {
    int ret;
    AVPacket pkt;
    int retry=0;
    while (retry++ <30) {
        ret = av_read_frame(ifmt_ctx_, &pkt);
        if (ret < 0) {
            printf("av_read_frame error!\n");
            break;
        }

        if (pkt.stream_index == videoindex_) {

            //判断是否是SEI
            if ((pkt.data[4] & 0x1F) == 6) {
                int k = 6;
                while (pkt.data[k - 1] != 0x54 || pkt.data[k] != 0x4D) {
                    k++;
                }
                k++;
                XSTime xs_sei_time;
                uint8_t datalenth = pkt.data[k];

                uint8_t xs_year1 = pkt.data[k + 1];
                uint8_t xs_year2 = pkt.data[k + 2];
                uint8_t xs_mon = pkt.data[k + 3];

                xs_sei_time.day_ = pkt.data[k + 4];
                xs_sei_time.hour_ = pkt.data[k + 5];
                xs_sei_time.min_ = pkt.data[k + 6];
                xs_sei_time.sec_ = pkt.data[k + 7];

                uint8_t xs_msec1 = pkt.data[k + 8];
                uint8_t xs_msec2 = pkt.data[k + 9];

                //xs_sei_time.msec_ = (pkt.data[k+8] <<8) +pkt.data[k+9];
                memcpy(&(xs_sei_time.msec_), pkt.data + k + 8, 2);

                // xs_sei_time.day_=0;
                // xs_sei_time.hour_=0;
                // xs_sei_time.min_=0;
                // xs_sei_time.sec_=0;
                // xs_sei_time.msec_=0;

                time = std::move(xs_sei_time);
                return true;
            }
        }
    }
    return false;
}

std::shared_ptr<Camera> CameraFactory::CreateCamera(CAMERA_TYPE cam_type, std::string path) {
    switch(cam_type){
        case CAMERA_TYPE_UVC:
            return std::make_shared<UVC>(path);
            break;
        case CAMERA_TYPE_XSCAM:
            return std::make_shared<UVC>(path,true);
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

//XSCAM::XSCAM(std::string path) {
//
//}
//
//bool XSCAM::GetImg(Ximg &img) {
//
//}
