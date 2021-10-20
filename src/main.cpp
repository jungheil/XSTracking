#include <iostream>

#include "dsst_tracker.hpp"
#include "tracker_run.hpp"

class DsstTrackerRun : public TrackerRun {
  public:
    DsstTrackerRun() : TrackerRun("DSSTcpp") {}

    virtual ~DsstTrackerRun() {}

    virtual cf_tracking::CfTracker* parseTrackerParas(
        bool enableDebug = false) {
        cf_tracking::DsstParameters paras;
        cv::FileStorage fs(_trackerConfigPath, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            fs.open("./trackerConfig.yaml", cv::FileStorage::READ);
            if (!fs.isOpened()) {
                std::cerr << "load param failed!" << std::endl;
                exit(100);
            }
        }
        if (!fs["templateSize"].empty())
            fs["templateSize"] >> paras.templateSize;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["padding"].empty())
            fs["padding"] >> paras.padding;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["pSigma"].empty())
            fs["pSigma"] >> paras.outputSigmaFactor;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["lambda"].empty())
            fs["lambda"] >> paras.lambda;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["learningRate"].empty())
            fs["learningRate"] >> paras.learningRate;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["hogCell"].empty())
            fs["hogCell"] >> paras.cellSize;
        else
            std::cerr << "missing parameters!" << std::endl;

        if (!fs["sSigma"].empty())
            fs["sSigma"] >> paras.scaleSigmaFactor;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["sStep"].empty())
            fs["sStep"] >> paras.scaleStep;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["sHogCell"].empty())
            fs["sHogCell"] >> paras.scaleCellSize;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["sNumOfScale"].empty())
            fs["sNumOfScale"] >> paras.numberOfScales;
        else
            std::cerr << "missing parameters!" << std::endl;

        if (!fs["psrThreshold"].empty())
            fs["psrThreshold"] >> paras.psrThreshold;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["peakDelSize"].empty())
            fs["peakDelSize"] >> paras.psrPeakDel;
        else
            std::cerr << "missing parameters!" << std::endl;
        if (!fs["enableLoss"].empty())
            fs["enableLoss"] >> paras.enableTrackingLossDetection;
        else
            std::cerr << "missing parameters!" << std::endl;

        fs.release();

        if (enableDebug) {
            setTrackerDebug(&_debug);
            return new cf_tracking::DsstTracker(paras, &_debug);
        } else {
            return new cf_tracking::DsstTracker(paras);
        }
    }

  private:
    cf_tracking::DsstDebug<cf_tracking::DsstTracker::T> _debug;
};

int main(int argc, const char** argv) {
    DsstTrackerRun mainObj;

    if (!mainObj.start()) return -1;

    return 0;
}
