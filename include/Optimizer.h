//
// Created by castoryan on 22.04.16.
//

#ifndef QR_SLAM_OPTIMIZER_H
#define QR_SLAM_OPTIMIZER_H




#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"


#include<mutex>
#include <g2o/types/slam3d/se3quat.h>

namespace QR_SLAM{

    class Optimizer {
    public:

        int static PoseOptimization(Frame *pFrame);

        g2o::SE3Quat static toSE3Quat(const cv::Mat &cvT);
        cv::Mat static toCvMat(const g2o::SE3Quat &SE3);
        cv::Mat static toCvMat(const Eigen::Matrix<double,4,4> &m);
    };
}


#endif //QR_SLAM_OPTIMIZER_H
