//
// Created by castoryan on 02.04.16.
//


#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <include/MapPoint.h>
#include <include/System.h>
#include "Initializer.h"


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


        usleep(50);

        //if(lastFrame.isKeyframe){
        //TrackWithTriangle();

        if(!initialized){
            MonocularInitialization();
        }
        else{
            //cout<< "Frame ID is "<<currentFrame.nThisId<<endl;

            cout<< "1 hahahaa" <<endl;
            bool trackOK = TrackReferenceKeyFrame();
            if(trackOK)
            {
                KeyFrame* kf= new KeyFrame(currentFrame);
                mCurrentRefKeyFrame = kf;
            }

            cout<< "2 hahahaa" <<endl;
            //need for keyframe insert
            //if(currentFrame.frameKeyFeatures.size()>7)
            //{
            //    KeyFrame kf_this(currentFrame);
              //  usSys->GlobalKeyFrame.push_back(&kf_this);
            //}
            //1. check if current frame is keyframe
            //2. check motion refer to the last keyframe
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
        for(int i = 0; i < fkf1.size(); i++)
        {

            for(int j = 0; j<fkf2.size(); j++)
            {
                if((fkf1[i].code_id == fkf2[j].code_id)&&
                        (fkf1[i].marker_id == fkf2[j].marker_id))
                {
                    kfpair12.push_back(j);
                }
            }
        }
        return kfpair12.size();
    }


    void Tracker::MonocularInitialization()
    {
        //Frame
        //currentIniFrame = currentFrame;


        if(!mpInitializer)
        {
            // Set Reference Frame
            if(currentFrame.frameKeyFeatures.size()>=8)
            {
                firstIniFrame  = Frame(currentFrame);



                if(mpInitializer)
                    delete mpInitializer;

                mpInitializer =  new Initializer(firstIniFrame,1.0,20);

                return;
            }
        }
        else
        {
            secondIniFrame = Frame(currentFrame);
            // Try to initialize
            if(secondIniFrame.frameKeyFeatures.size()<=8)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                return;
            }


            int nmatches = matchFeatures(firstIniFrame.frameKeyFeatures,
                                         secondIniFrame.frameKeyFeatures,
                                         featureMatches12);

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
            //vector<cv::Point3f> mvIniP3D;

            mpInitializer->Initialize(secondIniFrame, featureMatches12, Rcw, tcw,
                                      mvIniP3D, vbTriangulated);



                // Set Frame Poses
            if((!Rcw.empty())&&(!tcw.empty())) {
                cout <<"Rcw is " << Rcw <<endl;
                cout <<"tcw is " << tcw <<endl;
                firstIniFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(Tcw.rowRange(0, 3).col(3));
                secondIniFrame.SetPose(Tcw);

                CreateInitialMapMonocular();
            }

        }
    }

    void Tracker::CreateInitialMapMonocular()
    {


        KeyFrame* kf1 = new KeyFrame(firstIniFrame);
        KeyFrame* kf2 = new KeyFrame(secondIniFrame);

        usSys->GlobalKeyFrame.push_back(kf1);
        usSys->GlobalKeyFrame.push_back(kf2);

        for(int i; i<mvIniP3D.size(); i++)
        {
            MapPoint* mp = new MapPoint(mvIniP3D[i]);
            // put map point into global map point
            usSys->GlobalMapPoint.push_back(mp);
            // put map point into keyframe;
            kf2->mFrameMapPoints.push_back(mp);
        }

        kf2->refKeyFrame = kf1;
        mCurrentRefKeyFrame = kf2;

        initialized = true;
        cout<<"initialized is "<< initialized <<endl;
    }


    int matchByMapPoints(Frame &frame, KeyFrame* keyframe, std::vector<MapPoint*>& mappair12)
    {

        int count = 0;
        std::vector<Frame::keyFeature> featureList1 = frame.frameKeyFeatures;
        std::vector<Frame::keyFeature> featureList2 = keyframe->keyframeKeyFeatures;

       // find mappoint in the keyframe

       for(int i = 0; i < featureList1.size(); i++)
       {
          for(int j = 0; j < featureList2.size(); j++)
          {
              if((featureList1[i].code_id==featureList2[j].code_id)&&
                      (featureList1[i].marker_id==featureList2[j].marker_id))
              {
                  //TODO: problem is here!!
                  //MapPoint* pMP =  keyframe->mFrameMapPoints[j];
                  //if(pMP != NULL){
                  //frame.mFrameMapPoints.push_back(pMP);
                  //mappair12.push_back(pMP);
                  count++;
                 // }
              }
              else{

              }
          }
       }

        return count;
    }



    bool Tracker::TrackReferenceKeyFrame()
    {
        std::vector<MapPoint*> match_current2ref;

       int nmatches = matchByMapPoints(currentFrame, mCurrentRefKeyFrame,match_current2ref);



        //if(nmatches < 8)
          //  return false;
/*
        currentFrame.mFrameMapPoints = vpMapPointMatches;
        currentFrame.SetPose(mLastFrame.mTcw);

        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        // N is the number of keypoints in frame.
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            // if the keypoint is a map point
            if(mCurrentFrame.mvpMapPoints[i])
            {
                // if the keypoint is a outlier
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        return nmatchesMap>=10;
        */
        return true;
    }


}

