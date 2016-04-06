//
// Created by castoryan on 02.04.16.
//

#ifndef QR_SLAM_FRAME_H
#define QR_SLAM_FRAME_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <zbar.h>

namespace QR_SLAM {
    class Frame {

    public:
        struct keyFeature{
            cv::KeyPoint kp;
            int marker_id;
            int code_id;
        };

        //typedef keyFeature kFeature;
    public:
        Frame();
        Frame(const cv::Mat& img, const cv::Mat& K);

        bool isKeyframe;

        void FindKeypoints(const cv::Mat& img);

        void SetPose(cv::Mat Tcw);
        void UpdatePoseMatrices();

        // Camera pose.
        cv::Mat mTcw;

   // private:

        cv::Mat scanimg;

        std::vector<Frame::keyFeature> frameKeyFeatures;

        cv::Mat mK;


        // Rotation, translation and camera center
        cv::Mat mRcw;
        cv::Mat mtcw;
        cv::Mat mRwc;
        cv::Mat mOw; //==mtwc
    };
}

#endif //QR_SLAM_FRAME_H
