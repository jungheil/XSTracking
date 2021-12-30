//
// Created by li on 20/8/2021.
//

#ifndef XSTRACKING_COMMON_H
#define XSTRACKING_COMMON_H
#include <stdint.h>

#include <iostream>
#include <list>
#include <opencv2/core.hpp>

//#include <opencv2/opencv.hpp>
typedef struct XSTime {
    uint8_t  day_;
    uint8_t  hour_;
    uint8_t  min_;
    uint8_t  sec_;
    uint16_t msec_;
} XSTime;

class Camera;

class Ximg {
  public:
    Ximg() = default;

    Ximg(Camera *cam, cv::Mat &&src, uint64_t seq = 0)
        : cam_(cam), img_(std::move(src)), seq_(seq){};

    Ximg(Camera *cam, cv::Mat &&src, XSTime time, uint64_t seq = 0)
        : cam_(cam), img_(std::move(src)), time_(time), seq_(seq){};

    inline cv::Mat &     get_cv_color() { return img_; };
    inline const XSTime &get_time() { return time_; };
    inline uint64_t      get_seq() const { return seq_; };
    inline const Camera *get_cam() { return cam_; };

  private:
    Camera const *cam_;
    cv::Mat       img_;
    XSTime        time_{};
    uint64_t      seq_ = 0;
};
enum CAMERA_TYPE {
    CAMERA_TYPE_UNKOWN = 0,
    CAMERA_TYPE_XSCAM  = 1,
    CAMERA_TYPE_UVC    = 2
};
class Camera {
  public:
    Camera() { std::cout << (int)cam_count_ << std::endl; };
    virtual bool       GetImg(Ximg &img) = 0;
    bool               operator>>(Ximg &img) { return GetImg(img); };
    inline CAMERA_TYPE GetCamType() { return cam_type_; };
    inline uint8_t     get_id() const { return cam_id_; };

  protected:
    CAMERA_TYPE    cam_type_ = CAMERA_TYPE_UNKOWN;
    uint8_t        cam_id_   = 255;
    static uint8_t cam_count_;
};

inline bool operator<=(const XSTime &a0, const XSTime &a1) {
    return a0.day_ < a1.day_ || a0.day_ == a1.day_ && a0.hour_ < a1.hour_ ||
           a0.hour_ == a1.hour_ && a0.min_ < a1.min_ ||
           a0.min_ == a1.min_ && a0.sec_ < a1.sec_ ||
           a0.sec_ == a1.sec_ && a0.msec_ <= a1.msec_;
}

class ImgCache {
  public:
    explicit ImgCache(int size = 20) { cache_ = std::list<Ximg>(size); };
    void PushImg(const Ximg &img) {
        cache_.push_back(img);
        cache_.pop_front();
    }
    void Get(XSTime time, Ximg &img) {
        auto p = cache_.rbegin();
        while (time <= p->get_time()) { p++; }
        img = *p;
    }

  private:
    std::list<Ximg> cache_;
};

template <std::size_t _Nm>
class ImgBuff {
  public:
    explicit ImgBuff() = default;
    void Push(const Ximg &img) {
        buff_[p_] = img;
        p_        = Next(p_);
        if (p_ == g_) g_ = Next(g_);
    }
    bool Get(Ximg &img, bool isnew = false) {
        if (g_ == p_) { return false; }
        if (isnew) {
            img = buff_[Prev(p_)];
            g_  = p_;
        } else {
            img = buff_[g_];
            g_  = Next(g_);
        }
        return true;
    }
    bool Get(Ximg &img, XSTime time) {
        std::size_t i = Prev(p_);
        while (i != p_) {
            if (time <= buff_[i].get_time()) {
                img = buff_[i];
                g_  = Next(i);
                return true;
            }
            i = Prev(i);
        }
        img = buff_[p_];
        g_  = Next(p_);
        return true;
    }
    std::size_t get_delay() {
        if (g_ <= p_) {
            return p_ - g_;
        } else {
            return p_ - g_ + _Nm;
        }
    }

  private:
    std::size_t Next(std::size_t p) {
        p = (p + 1) % _Nm;
        return p;
    }
    std::size_t Prev(std::size_t p) {
        p = (p - 1) % _Nm;
        return p;
    }

  private:
    std::array<Ximg, _Nm> buff_;
    std::size_t           p_ = 0;
    std::size_t           g_ = 0;
};
#include <ctime>
#include <opencv2/videoio.hpp>
class VideoRecorder {
  public:
    VideoRecorder(std::string path, cv::Size size, bool isColor) : path_(path) {
        st_ = clock();
        //        writer.open(path,cv::VideoWriter::fourcc('D', 'I', 'V', 'X')
        //        ,30,size,isColor);
    };
    inline bool operator<<(const cv::Mat &src) {
        //        if((clock()-st_)/CLOCKS_PER_SEC));
        if (!videoWriter_) {
            time_t t;
            time(&t);
            char *      str  = ctime(&t);
            std::string path = path_ + str + ".avi";
            videoWriter_     = std::make_unique<cv::VideoWriter>(
                path, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 30, size_,
                isColor_);
        }
        (*videoWriter_) << src;
    }
    void Release() {
        videoWriter_->release();
        videoWriter_.reset();
    }

  private:
    std::string                      path_;
    cv::Size                         size_;
    bool                             isColor_;
    std::unique_ptr<cv::VideoWriter> videoWriter_ = nullptr;
    //    cv::VideoWriter writer;
    time_t st_;
};

#endif  // XSTRACKING_COMMON_H
