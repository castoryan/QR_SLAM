//
// Created by castoryan on 02.04.16.
//

#ifndef QR_SLAM_VIEWER_H
#define QR_SLAM_VIEWER_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

using namespace std;

namespace QR_SLAM {
    class MapPoint;
    class KeyFrame;
    class System;
    class Tracker;

    class Viewer {
    public:
        Viewer(vector<MapPoint*> GlobalMapPoint,vector<KeyFrame*> GlobalKeyFrame);

       // void DrawMapPoints();
        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void SetCurrentCameraPose(const cv::Mat &Twc);
        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

        void Run();



        System* mpSystem;
        Tracker* mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        bool mbStopped;
        bool mbStopRequested;

        cv::Mat mCameraPose;

        vector<MapPoint*> mps;
        vector<KeyFrame*> kfs;
    };
}

#endif //QR_SLAM_VIEWER_H
