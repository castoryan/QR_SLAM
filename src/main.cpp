#include <iostream>
#include <opencv2/opencv.hpp>
#include <include/System.h>
#include <unistd.h>


using namespace std;
using namespace cv;

int main() {
    cout << "QR-SLAM is Starting, version 0.01" << endl;

    VideoCapture capture(0);

    QR_SLAM::System sys(1);



    usleep(50000);

    return 0;
}