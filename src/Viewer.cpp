//
// Created by castoryan on 02.04.16.
//

#include <unistd.h>
#include "Viewer.h"
#include <include/KeyFrame.h>

namespace QR_SLAM{

    Viewer::Viewer(vector<MapPoint*> GlobalMapPoint, vector<KeyFrame*> GlobalKeyFrame):
    mps(GlobalMapPoint), kfs(GlobalKeyFrame)
    {

        float fps = 20;
        if(fps<1)
            fps=30;
        mT = 1e3/fps;

        mImageWidth =640;
        mImageHeight = 480;
        if(mImageWidth<1 || mImageHeight<1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = 0;
        mViewpointY = -10;
        mViewpointZ = -0.1;
        mViewpointF = 2000;

        mKeyFrameSize = 0.1;
        mKeyFrameLineWidth = 1;
        mGraphLineWidth = 1;
        mPointSize = 2;
        mCameraSize = 0.15;
        mCameraLineWidth = 2;

    }

    void Viewer::Run(){


        pangolin::CreateWindowAndBind("QR-SLAM",1024,768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
       // pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        //pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        //pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        //pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
        //pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
        //pangolin::Var<bool> menuReset("menu.Reset",false,false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        bool bFollow = true;
        bool bLocalizationMode = false;



        while(1){
            GetCurrentOpenGLCameraMatrix(Twc);
            s_cam.Follow(Twc);

            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            DrawCurrentCamera(Twc);
            DrawKeyFrames(true, true);



            pangolin::FinishFrame();

            // std::cout<<"in Viewer"<<std::endl;


            // draw MapPoints

            // draw KeyFrames

            usleep(10000);
        }

    }

/*
    void Viewer::DrawMapPoints()
    {
        const vector<MapPoint*> &vpMPs = mps;
        const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

        std::set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if(vpMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,0.0);

        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            cv::Mat pos = vpMPs[i]->my_world_position;
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0,0.0,0.0);

        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
                continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

        }

        glEnd();
    }*/

    void Viewer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
    {
        const float &w = mKeyFrameSize;
        const float h = w*0.75;
        const float z = w*0.6;

        const vector<KeyFrame*> vpKFs = kfs;

        if(bDrawKF)
        {
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                cv::Mat Twc = pKF->Twc.t();

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f,0.0f,1.0f);
                glBegin(GL_LINES);
                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
/*
        if(bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f,1.0f,0.0f,0.6f);
            glBegin(GL_LINES);

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if(!vCovKFs.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                    {
                        if((*vit)->mnId<vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame* pParent = vpKFs[i]->GetParent();
                if(pParent)
                {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
                }


            }

            glEnd();
        }
        */
    }

    void Viewer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
    {
        const float &w = mCameraSize;
        const float h = w*0.75;
        const float z = w*0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }


    void Viewer::SetCurrentCameraPose(const cv::Mat &Tcw)
    {
        mCameraPose = Tcw.clone();
    }

    void Viewer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        if(!mCameraPose.empty())
        {
            cv::Mat Rwc(3,3,CV_32F);
            cv::Mat twc(3,1,CV_32F);
            {
                Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
                twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
            }

            M.m[0] = Rwc.at<float>(0,0);
            M.m[1] = Rwc.at<float>(1,0);
            M.m[2] = Rwc.at<float>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc.at<float>(0,1);
            M.m[5] = Rwc.at<float>(1,1);
            M.m[6] = Rwc.at<float>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc.at<float>(0,2);
            M.m[9] = Rwc.at<float>(1,2);
            M.m[10] = Rwc.at<float>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15]  = 1.0;
        }
        else
            M.SetIdentity();
    }

}