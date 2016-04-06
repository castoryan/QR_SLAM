//
// Created by castoryan on 06.04.16.
//

#ifndef QR_SLAM_INITIALIZER_H
#define QR_SLAM_INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"

//#include<vector>

using namespace std;
namespace QR_SLAM {

    class Initializer
    {


    public:
        typedef pair<int, int> Match;

        // Fix the reference frame
        Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

        // Computes in parallel a fundamental matrix and a homography
        // Selects a model and tries to recover the motion and the structure from motion
        bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                        cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


    private:

        void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
        void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

        cv::Mat ComputeH21(const vector<Frame::keyFeature> &vP1, const vector<Frame::keyFeature> &vP2);
        cv::Mat ComputeF21(const vector<Frame::keyFeature> &vP1, const vector<Frame::keyFeature> &vP2);

        float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

        float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

        bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                          cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

        void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

        void Normalize(const vector<Frame::keyFeature> &vKeys, vector<Frame::keyFeature> &vNormalizedPoints, cv::Mat &T);

        int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<Frame::keyFeature> &vKeys1, const vector<Frame::keyFeature> &vKeys2,
                    const vector<Match> &vMatches12, vector<bool> &vbInliers,
                    const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

        void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


        // Keypoints from Reference Frame (Frame 1)
        vector<Frame::keyFeature> mvKeys1;

        // Keypoints from Current Frame (Frame 2)
        vector<Frame::keyFeature> mvKeys2;

        // Current Matches from Reference to Current
        vector<Match> mvMatches12;
        vector<bool> mvbMatched1;

        // Calibration
        cv::Mat mK;

        // Standard Deviation and Variance
        float mSigma, mSigma2;

        // Ransac max iterations
        int mMaxIterations;

        // Ransac sets
        vector<vector<size_t> > mvSets;

    };
}

#endif //QR_SLAM_INITIALIZER_H
