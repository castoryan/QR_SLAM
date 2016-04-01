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

namespace QR_SLAM{

    class Viewer;
    class Mapper;
    class Loopper;



    class System {

    public:
        System(int a);
        void TrackMono(cv::Mat);

        std::thread* MapperTh;
        std::thread* ViewerTh;
        std::thread* LoopperTh;

        Viewer* ViewerRunning;
        Mapper* MapperRunning;
        Loopper* LoopperRunning;

    private:
    };
}


#endif //QR_SLAM_SYSTEM_H
