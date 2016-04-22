//
// Created by castoryan on 02.04.16.
//

#ifndef QR_SLAM_KEYFRAME_H
#define QR_SLAM_KEYFRAME_H

#include <opencv2/opencv.hpp>
#include "Frame.h"


namespace QR_SLAM {

    class Frame;


    class KeyFrame {



    public:
        KeyFrame();
        KeyFrame(const Frame &frame);


    public:
        void SetPose(cv::Mat Tcw);
        void UpdatePoseMatrices();
        std::vector<Frame::keyFeature>  GetFeatures();


        long unsigned int nNextKFId;
        long unsigned int nKeyFrameId;

        long unsigned int nFrameId;

        // Camera pose.
        cv::Mat mTcw;

        // private:
        cv::Mat mK;

        // Reference Keyframe of this one
        KeyFrame* refKeyFrame;

        // Feautres in this keyframe
        std::vector<Frame::keyFeature> keyframeKeyFeatures;

        // MapPoints associated to keypoints, NULL pointer if no association.
        std::vector<MapPoint*> mFrameMapPoints;

        // SE3 Pose and camera center
        cv::Mat Tcw;
        cv::Mat Twc;
        cv::Mat Ow;

        //cv::Mat
    };
}

#endif //QR_SLAM_KEYFRAME_H
