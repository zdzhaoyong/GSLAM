#include "gtest.h"

#ifdef HAS_EIGEN3
#include <Eigen/Core>
#endif

#include "../../core/GSLAM.h"
#include "../../core/Event.h"

TEST(SLAM,LoadRelease)
{
    std::string slamPlugin=svar.GetString("SLAM","");
    if(slamPlugin.empty()) return;
    GSLAM::SLAMPtr slam=GSLAM::SLAM::create(slamPlugin);
    slam->call("Stop");
    slam.reset();
}

#ifdef EIGEN_MATRIX_H

TEST(SLAM,EigenPoint)
{
    Eigen::Vector2d pt2_eigen;
    GSLAM::Point2f  pt2_gslam=pt2_eigen;
    pt2_eigen=pt2_gslam;

    Eigen::Vector3d pt3_eigen;
    GSLAM::Point3f  pt3_gslam=pt3_eigen;
    pt3_eigen=pt3_gslam;

    pi::SE3f      se3;
    Eigen::Matrix<float,6,1> ln=se3.ln();
}

#endif
