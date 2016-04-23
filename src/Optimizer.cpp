//
// Created by castoryan on 22.04.16.
//

#include "Optimizer.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
//#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;

using namespace std;

namespace QR_SLAM{


    int Optimizer::PoseOptimization(Frame *pFrame)
    {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        //g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType> ();

        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences=0;

        // Set Frame vertex
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        g2o::SE3Quat pose_Qu = toSE3Quat(pFrame->mTcw);
        vSE3->setEstimate(pose_Qu);
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);


        // Set MapPoint vertices
        const int N = pFrame->mFrameMapPoints.size();

        vector<g2o::EdgeProjectXYZ2UV*> vpEdgesMono;
        vector<size_t> vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        const float deltaMono = sqrt(5.991);


        for(int i=0; i<N; i++)
        {
            MapPoint *pMP = pFrame->mFrameMapPoints[i];

            // add map points into optimizer
            Eigen::Matrix<double,3,1> obsv;
            const cv::Point3f &kp = pMP->my_world_postition_point3f;

            cout << "kp is " << kp.x << " y is "<< kp.y <<" z is "<< kp.z << endl;
//            cout << "obsv is " << obsv << endl;
/*                 obsv << static_cast<double>(kp.x),
                        static_cast<double>(kp.y),
                        static_cast<double>(kp.z);
                g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();


                v->setEstimate(obsv);
                v->setId(i+1);
                optimizer.addVertex(v);
*/
        }


        for(int i=0; i<N; i++)
        {
            MapPoint* pMP = pFrame->mFrameMapPoints[i];
            if(pMP)
            {
                nInitialCorrespondences++;


                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->frameKeyFeatures[i].kp;
                obs << kpUn.pt.x, kpUn.pt.y;


                // add reprojection error into optimizer
                g2o::EdgeProjectXYZ2UV* e = new g2o::EdgeProjectXYZ2UV();


                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                e->setMeasurement(obs);
                e->setInformation(Eigen::Matrix2d::Identity());


                //e->_cam->principle_point << cx, cy;
                //e->_cam->focal_length = fx;

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);


                bool added = optimizer.addEdge(e);
                //cout << "add ed is "<< added << endl;

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back(i);
            }

        }



        if(nInitialCorrespondences<3)
            return 0;

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4]={5.991,5.991,5.991,5.991};
        const int its[4]={10,10,10,10};

        int nBad=0;
        for(size_t it=0; it<4; it++)
        {
            vSE3->setEstimate(toSE3Quat(pFrame->mTcw));
            optimizer.setVerbose(true);
            bool iniop = optimizer.initializeOptimization(0);
            bool doop = optimizer.optimize(its[it],true);


            nBad=0;
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeProjectXYZ2UV* e = vpEdgesMono[i];
                const size_t idx = vnIndexEdgeMono[i];
                //e->computeError();
                const float chi2 = e->chi2();

                if(it==2)
                    e->setRobustKernel(0);
            }

            if(optimizer.edges().size()<10);
                //break;
        }

        // Recover optimized pose and return number of inliers
        g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = toSE3Quat(toCvMat(vSE3_recov->estimate()));
        cv::Mat pose = toCvMat(SE3quat_recov);
        pFrame->SetPose(pose);

        return nInitialCorrespondences-nBad;
    }


    g2o::SE3Quat Optimizer::toSE3Quat(const cv::Mat &cvT)
    {
        Eigen::Matrix<double,3,3> R;
        R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
                cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
                cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

        Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

        return g2o::SE3Quat(R,t);
    }

    cv::Mat Optimizer::toCvMat(const Eigen::Matrix<double,4,4> &m)
    {
        cv::Mat cvMat(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<float>(i,j)=m(i,j);

        return cvMat.clone();
    }

    cv::Mat Optimizer::toCvMat(const g2o::SE3Quat &SE3)
    {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
        return toCvMat(eigMat);
    }

}