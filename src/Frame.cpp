//
// Created by castoryan on 02.04.16.
//


#include "Frame.h"
#include <zbar.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace zbar;

namespace QR_SLAM{

    Frame::Frame(){

    };


    Frame::Frame(const cv::Mat& img){

        FindKeypoints(img);

    }

    void Frame::FindKeypoints(const cv::Mat& img){


        scanimg = img;




        Image qrimage(scanimg.cols, scanimg.rows, "Y800", scanimg.data, scanimg.cols* scanimg.rows);


        if( scanner.scan(qrimage) > 0){
            for(Image::SymbolIterator symbol = qrimage.symbol_begin();symbol !=
                                                                      qrimage.symbol_end();
                ++symbol)
            {
                for(int k = 0; k < 4; k++)if(symbol->get_location_size()==4){
                        //std::cout << "locations are "  <<  symbol->get_location_x(k) <<" "<<symbol->get_location_y(k)<<std::endl;
                        cv::circle( scanimg, cv::Point( symbol->get_location_x(k), symbol->get_location_y(k) ), 4, cv::Scalar( 0, 0, 255 ), 3, 8 );
                    }



                std::string out = "";
                char array[10];
                out = symbol->get_data() + '('+ array + ')';
               // std::cout << out<< std::endl;
                cv::imshow("123",scanimg);
            }
        }

    }

}