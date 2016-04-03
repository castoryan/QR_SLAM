//
// Created by castoryan on 01.04.16.
//

#include "System.h"


using namespace std;

namespace QR_SLAM{

    System::System(int a){
        std::cout << "QR-SLAM is Starting, version 0.01" << std::endl;

        TrackerRunning = new Tracker(this);


        MapperRunning = new Mapper();
        MapperTh  = new thread(&QR_SLAM::Mapper::Run, MapperRunning);
        LoopperRunning = new Loopper();
        LoopperTh = new thread(&QR_SLAM::Loopper::Run, LoopperRunning);
        ViewerRunning = new Viewer();
        ViewerTh  = new thread(&QR_SLAM::Viewer::Run, ViewerRunning);

    };

    cv::Mat System::TrackMono(cv::Mat img) {

        return TrackerRunning->GetNewImg(img);
    }
}
