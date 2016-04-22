//
// Created by castoryan on 02.04.16.
//

#ifndef QR_SLAM_TRACKER_H
#define QR_SLAM_TRACKER_H

#include <opencv2/core/core.hpp>
#include "Frame.h"
#include "KeyFrame.h"

namespace QR_SLAM {

    class System;
    class Initializer;


    class Tracker {

    public:
        Tracker(System* sys);
        cv::Mat GetNewImg(const cv::Mat &img);
        void Track();
        void TrackWithTriangle();
        int matchFeatures(std::vector<Frame::keyFeature> fkf1, std::vector<Frame::keyFeature>fkf2, std::vector<int> &kfpair12);
        void MonocularInitialization();
        void CreateInitialMapMonocular();
        bool TrackReferenceKeyFrame();


    public:
        Frame currentFrame;
        Frame lastFrame;

        Frame firstIniFrame;
        Frame secondIniFrame;

        KeyFrame* mCurrentRefKeyFrame;

        std::vector<cv::Point3f> mvIniP3D;

    private:
        System* usSys;
        bool inifRGB;
        cv::Mat imGray;


        bool initialized;


        // camera inside parameter
        cv::Mat mK;


        //For Initializing
        Initializer* mpInitializer;
        std::vector<Frame::keyFeature> preMatched;

        // the one feature in frame1 correspond to which number in frame2
        std::vector<int> featureMatches12;
    };


}


#endif //QR_SLAM_TRACKER_H
