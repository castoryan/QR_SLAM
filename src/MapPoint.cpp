//
// Created by castoryan on 06.04.16.
//

#include <include/MapPoint.h>

namespace QR_SLAM{

    MapPoint::MapPoint()
    {
    }

    MapPoint::MapPoint(cv::Point3f Pos){

        my_world_postition_point3f = Pos;
        my_world_postition = cv::Mat(Pos);
    }
    void MapPoint::addKeyFrame(KeyFrame& kf){
        size_t st = 5;
        my_observation.count(&kf);
    }


}

