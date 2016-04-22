//
// Created by castoryan on 01.04.16.
//

#ifndef QR_SLAM_SYSTEM_H
#define QR_SLAM_SYSTEM_H


#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "Mapper.h"
#include "Viewer.h"
#include "Loopper.h"
#include "Tracker.h"

namespace QR_SLAM{

    class Viewer;
    class Mapper;
    class Loopper;
    class Tracker;
    class KeyFrame;
    class MapPoint;

    class System {

    public:
        System(int a);
        cv::Mat TrackMono(cv::Mat);

        std::thread* MapperTh;
        std::thread* ViewerTh;
        std::thread* LoopperTh;

        Tracker* TrackerRunning;

        Viewer* ViewerRunning;
        Mapper* MapperRunning;
        Loopper* LoopperRunning;

        std::vector<KeyFrame*> GlobalKeyFrame;
        std::vector<MapPoint*> GlobalMapPoint;

    private:
    };
}


#endif //QR_SLAM_SYSTEM_H
