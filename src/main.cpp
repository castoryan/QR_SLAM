#include <iostream>
#include <opencv2/opencv.hpp>
#include <include/System.h>



using namespace std;
using namespace cv;

int main() {
    //cout << "QR-SLAM is Starting, version 0.01" << endl;

    VideoCapture capture(0);

    QR_SLAM::System sys(1);
/*
    usleep(100000);
    int count = 0;
    Mat img;
    for(int i; i < 82; i++){
        count++;
        string num = to_string(count);
        string loc = "/home/castoryan/QR_SLAM/data/" + num + ".png";
        img = imread(loc);
        imshow("img", img);

        sys.TrackMono(img);



       waitKey(50);
    }
*/

    int count = 0;
    Mat img;
    for(;;)
    {
        capture>>img;
        sys.TrackMono(img);

        imshow("Image",img);


        //count++;
        //string num = to_string(count);
        //string file = "/home/castoryan/QR_SLAM/data/"+num+".png";
        //imwrite(file,img);
        waitKey(30);
    }

    //capture.release();
    //usleep(50000);

    return 0;
}