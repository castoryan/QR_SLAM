//
// Created by castoryan on 02.04.16.
//

#ifndef QR_SLAM_TRACKER_H
#define QR_SLAM_TRACKER_H

#include <opencv2/core/core.hpp>
#include "Frame.h"

namespace QR_SLAM {

    class System;


    class Tracker {

    public:
        Tracker(System* sys);
        cv::Mat GetNewImg(const cv::Mat &img);
        void Track();


    public:
        Frame currentFrame;

    private:
        System* usSys;
        bool inifRGB;
        cv::Mat imGray;
    };


}


#endif //QR_SLAM_TRACKER_H
