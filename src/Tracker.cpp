//
// Created by castoryan on 02.04.16.
//


#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <include/Initializer.h>
#include "Tracker.h"


namespace QR_SLAM {


    Tracker::Tracker(System* sys):
            inifRGB(true), usSys(sys), mpInitializer((static_cast<Initializer*>(NULL)))
    {
        float fx = 318.856;
        float fy = 318.856;
        float cx = 307.1928;
        float cy = 245.2157;


        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        K.copyTo(mK);

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
        currentFrame = Frame(imGray,mK);
        Track();
        return img;
    }

    void Tracker::Track(){

        if(lastFrame.isKeyframe){
        //TrackWithTriangle();
            MonocularInitialization();
        }

        lastFrame = currentFrame;
    }

    void Tracker::TrackWithTriangle(){
        lastFrame.frameKeyFeatures;
        currentFrame.frameKeyFeatures;
    }

    int Tracker::matchFeatures(std::vector<Frame::keyFeature> fkf1, std::vector<Frame::keyFeature> fkf2, std::vector<int>& kfpair12)
    {

        kfpair12.clear();
        for(int i = 0; i < fkf1.size(); i++){

            for(int j = 0; j<fkf2.size(); j++){
                if((fkf1[i].code_id == fkf2[j].code_id)&&(fkf1[i].marker_id == fkf2[j].marker_id)){

                    kfpair12.push_back(j);
                }
            }

        }
        return kfpair12.size();
    }


    void Tracker::MonocularInitialization()
    {
        currentIniFrame = currentFrame;


        if(!mpInitializer)
        {
            // Set Reference Frame
            if(currentIniFrame.frameKeyFeatures.size()>=8)
            {
                //std::cout <<"1 FEATURE SIZE is "<< currentIniFrame.frameKeyFeatures.size() <<endl;

                currentIniFrame = Frame(currentIniFrame);
                lastIniFrame = Frame(currentIniFrame);
                //for(size_t i=0; i<currentIniFrame.frameKeyFeatures.size(); i++)
                  //  preMatched[i] = currentIniFrame.frameKeyFeatures[i];

                if(mpInitializer)
                    delete mpInitializer;

                mpInitializer =  new Initializer(currentIniFrame,1.0,20);

                return;
            }
        }
        else
        {
            // Try to initialize
            if((int)currentIniFrame.frameKeyFeatures.size()<=8)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                return;
            }

            //std::cout <<"2 FEATURE SIZE is "<< currentIniFrame.frameKeyFeatures.size() <<endl;


            int nmatches = matchFeatures(lastIniFrame.frameKeyFeatures,
                                         currentIniFrame.frameKeyFeatures,
                                         featureMatches12);

            std::cout <<"the num of nmatches  are "<<nmatches<<endl;

            // Check if there are enough correspondences
            if(nmatches<8)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                return;
            }



            cv::Mat Rcw; // Current Camera Rotation
            cv::Mat tcw; // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
            vector<cv::Point3f> mvIniP3D;

            mpInitializer->Initialize(currentIniFrame, featureMatches12, Rcw, tcw,
                                      mvIniP3D, vbTriangulated);


            cout <<"Rcw is " << Rcw <<endl;
            cout <<"tcw is " << tcw <<endl;
                // Set Frame Poses
            if((!Rcw.empty())&&(!tcw.empty())) {
                lastIniFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(Tcw.rowRange(0, 3).col(3));
                currentIniFrame.SetPose(Tcw);
            }
                //CreateInitialMapMonocular();
        }
    }



}

