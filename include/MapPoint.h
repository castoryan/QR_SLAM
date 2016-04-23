//
// Created by castoryan on 06.04.16.
//

#ifndef QR_SLAM_MAPPOINT_H
#define QR_SLAM_MAPPOINT_H

#include <opencv2/opencv.hpp>
#include "KeyFrame.h"

namespace QR_SLAM {

    class KeyFrame;

    class MapPoint {

    public:
        MapPoint();
        MapPoint(cv::Point3f worldPos);


        void addKeyFrame(KeyFrame& kf);
        // coordinator in the world axis
        cv::Mat my_world_postition;
        cv::Point3f my_world_postition_point3f;

        // Keyframes that observed the map point and its index
        std::map<KeyFrame*,size_t> my_observation;


    };


}

#endif //QR_SLAM_MAPPOINT_H
