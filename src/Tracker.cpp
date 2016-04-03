//
// Created by castoryan on 02.04.16.
//


#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "Tracker.h"


namespace QR_SLAM {


    Tracker::Tracker(System* sys):
            inifRGB(true), usSys(sys)
    {

    }


    cv::Mat Tracker::GetNewImg(const cv::Mat &img){



        imGray = img;

        if(imGray.channels()==3)
        {
            if(inifRGB)
                cv::cvtColor(imGray,imGray,CV_RGB2GRAY);
            else
                cv::cvtColor(imGray,imGray,CV_BGR2GRAY);
        }
        else if(imGray.channels()==4)
        {
            if(inifRGB)
                cv::cvtColor(imGray,imGray,CV_RGBA2GRAY);
            else
                cv::cvtColor(imGray,imGray,CV_BGRA2GRAY);
        }

        //std::cout<<"to currentFrame"<<std::endl;
        currentFrame = Frame(imGray);
        Track();
        return img;
    }

    void Tracker::Track(){

    }

}