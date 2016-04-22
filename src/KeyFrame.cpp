//
// Created by castoryan on 02.04.16.
//

#include "KeyFrame.h"

using namespace std;

namespace QR_SLAM{

    long unsigned int nNextKFId = 0;

    KeyFrame::KeyFrame(){
    }

    KeyFrame::KeyFrame(const Frame &frame):
            nKeyFrameId(frame.nThisId),keyframeKeyFeatures(frame.frameKeyFeatures)
    {
        nKeyFrameId = nNextKFId++;
        SetPose(frame.mTcw);
    }

    void KeyFrame::SetPose(cv::Mat Tcw_)
    {
       Tcw_.copyTo(Tcw);
        if(!Tcw.empty())
        {
            cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = Tcw.rowRange(0,3).col(3);
            cv::Mat Rwc = Rcw.t();
            Ow = -Rwc*tcw;

            Twc = cv::Mat::eye(4,4,Tcw.type());
            Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
            Ow.copyTo(Twc.rowRange(0,3).col(3));
        }

    }

    vector<Frame::keyFeature> KeyFrame::GetFeatures()
    {
        return keyframeKeyFeatures;
    }

}