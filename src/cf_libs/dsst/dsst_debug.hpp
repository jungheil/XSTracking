
#ifndef DSST_DEBUG_HPP_
#define DSST_DEBUG_HPP_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#ifndef TERMINAL_MODE
#include "opencv2/highgui.hpp"
#endif
#include <fstream>
#include <iostream>

#include "tracker_debug.hpp"

namespace cf_tracking {
template <typename T>
class DsstDebug : public TrackerDebug {
  public:
    DsstDebug() : _maxResponse(0), _psrClamped(0), _targetSizeArea(0) {}

    virtual ~DsstDebug() {
        if (_outputFile.is_open()) _outputFile.close();
    }

    virtual void init(std::string outputFilePath) {
#ifndef TERMINAL_MODE
        namedWindow(_SUB_WINDOW_TITLE, cv::WINDOW_NORMAL);
        namedWindow(_RESPONSE_TITLE, cv::WINDOW_NORMAL);
#endif
        _outputFile.open(outputFilePath.c_str());
    }

    virtual void printOnImage(cv::Mat& image) {
        _ss.str("");
        _ss.clear();
        _ss << "Max Response: " << _maxResponse;
        putText(image, _ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_TRIPLEX,
                0.5, cv::Scalar(255, 0, 0));

        _ss.str("");
        _ss.clear();
        _ss << "PSR Clamped: " << _psrClamped;
        putText(image, _ss.str(), cv::Point(20, 80), cv::FONT_HERSHEY_TRIPLEX,
                0.5, cv::Scalar(255, 0, 0));

        _ss.str("");
        _ss.clear();
        _ss << "Area: " << _targetSizeArea;
        putText(image, _ss.str(), cv::Point(image.cols - 100, 80),
                cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 0, 0));
    }

    virtual void printConsoleOutput() {}

    virtual void printToFile() {
        _outputFile << _maxResponse << "," << _psrClamped << std::endl;
    }

    void showPatch(const cv::Mat& patchResized) {
#ifndef TERMINAL_MODE
        imshow(_SUB_WINDOW_TITLE, patchResized);
#endif
    }

    void setPsr(double psrClamped) { _psrClamped = psrClamped; }

    void showResponse(const cv::Mat& response, double maxResponse) {
        cv::Mat responseOutput = response.clone();
        _maxResponse           = maxResponse;
#ifndef TERMINAL_MODE
        imshow(_RESPONSE_TITLE, responseOutput);
#endif
    }

    void setTargetSizeArea(T targetSizeArea) {
        _targetSizeArea = targetSizeArea;
    }

  private:
    const std::string _SUB_WINDOW_TITLE = "Sub Window";
    const std::string _RESPONSE_TITLE   = "Response";
    double            _maxResponse;
    double            _psrClamped;
    T                 _targetSizeArea;
    std::stringstream _ss;
    std::ofstream     _outputFile;
};
}  // namespace cf_tracking

#endif
