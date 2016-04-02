//
// Created by castoryan on 02.04.16.
//

#ifndef QR_SLAM_FRAME_H
#define QR_SLAM_FRAME_H

#include <opencv2/core/core.hpp>

namespace QR_SLAM {
    class Frame {

    public:
        Frame();
        Frame(const cv::Mat& img);

        void FindKeypoints(const cv::Mat& img);

    private:

        cv::Mat scanimg;
    };
}

#endif //QR_SLAM_FRAME_H
