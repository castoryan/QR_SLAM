//
// Created by castoryan on 02.04.16.
//


#include "Frame.h"
//#include <opencv2/highgui/highgui.hpp>
#include <chrono>


using namespace zbar;

namespace QR_SLAM{
    long unsigned int nThisId = 0;
    long unsigned int nNextId = 0;

    Frame::Frame():
            isKeyframe(false)
    {

    };

    Frame::Frame(const Frame &frame):
            nThisId(frame.nThisId), frameKeyFeatures(frame.frameKeyFeatures), mK(frame.mK),mTcw(frame.mTcw), isKeyframe(frame.isKeyframe)
    {

    }


    Frame::Frame(const cv::Mat& img, const cv::Mat& K):
            isKeyframe(false), mK(K)
    {
        nThisId = nNextId++;
        FindKeypoints(img);
    }


    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0,3).colRange(0,3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0,3).col(3);
        mOw = -mRcw.t()*mtcw;
    }


    void Frame::FindKeypoints(const cv::Mat& img)
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        scanimg = img;
        int kPcount = 0;

        zbar::ImageScanner scanner;
        scanner.set_config( zbar::ZBAR_NONE,  zbar::ZBAR_CFG_ENABLE, 1);

        Image qrimage(scanimg.cols, scanimg.rows, "Y800", scanimg.data, scanimg.cols* scanimg.rows);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        if( scanner.scan(qrimage) > 0){
            for(Image::SymbolIterator symbol = qrimage.symbol_begin();symbol !=
                                                                      qrimage.symbol_end();
                ++symbol)
            {
                kPcount++;
                for(int k = 0; k < 4; k++)if(symbol->get_location_size()==4){

                        keyFeature kfeature;

                        cv::KeyPoint kp;
                        kp.pt.x = (float)symbol->get_location_x(k);
                        kp.pt.y = (float)symbol->get_location_y(k);

                        kfeature.kp = kp;
                        kfeature.marker_id = k;
                        kfeature.code_id =  stoi(symbol->get_data());
                        frameKeyFeatures.push_back(kfeature);

                        //std::cout << "locations are "  <<  symbol->get_location_x(k) <<" "<<symbol->get_location_y(k)<<std::endl;
                        cv::Point sympos =  cv::Point( symbol->get_location_x(k), symbol->get_location_y(k));
                        cv::circle( scanimg, sympos, 4, cv::Scalar( 0, 0, 255 ), 3, 8 );
                    }


                std::string out =  "the id of code is " + symbol->get_data();
                //std::cout << out<< std::endl;
                //cv::imshow("123", scanimg);
            }

            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

            double ttrack1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
            double ttrack2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
            double ttrack3 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t1).count();
            //std::cout <<"ttrack1 "<< ttrack1 <<"us ttrack2 "<< ttrack2 <<"us ttrack3 is "<< ttrack3 <<"us"<< std::endl;
            isKeyframe = true;
        }
    }

}