#include <iostream>
#include <opencv2/opencv.hpp>
#include <include/System.h>


using namespace std;
using namespace cv;

int main() {
    cout << "QR-SLAM is Starting, version 0.01" << endl;

    VideoCapture capture(1);
    //capture.release();
    QR_SLAM::System sys(1);

    Mat img;
    for(;;){
        capture>>img;
        sys.TrackMono(img);


        //imshow("Image",img);
        waitKey(30);
    }

    //usleep(50000);

    return 0;
}