//
// Created by castoryan on 02.04.16.
//


#include "Frame.h"
#include <iostream>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>


using namespace zbar;

namespace QR_SLAM{

    Frame::Frame(){

    };


    Frame::Frame(const cv::Mat& img){

        FindKeypoints(img);
    }

    void Frame::FindKeypoints(const cv::Mat& img){


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

                        cv::KeyPoint kp;
                        kp.pt.x = symbol->get_location_x(k);
                        kp.pt.y = symbol->get_location_y(k);
                        frameKeyPoints[kPcount] = kp;

                        std::cout << "locations are "  <<  symbol->get_location_x(k) <<" "<<symbol->get_location_y(k)<<std::endl;
                        cv::Point sympos =  cv::Point( symbol->get_location_x(k), symbol->get_location_y(k));
                        cv::circle( scanimg, sympos, 4, cv::Scalar( 0, 0, 255 ), 3, 8 );
                    }



                std::string out =  "the id of code is " + symbol->get_data();
                std::cout << out<< std::endl;
                cv::imshow("123",scanimg);
            }

            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

            double ttrack1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
            double ttrack2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
            double ttrack3 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t1).count();
            std::cout <<"ttrack1 "<< ttrack1 <<"us ttrack2 "<< ttrack2 <<"us ttrack3 is "<< ttrack3 <<"us"<< std::endl;
        }




    }

}