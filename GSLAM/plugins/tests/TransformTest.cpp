#include <GSLAM/core/GSLAM.h>
#include "gtest.h"

using namespace GSLAM;
using namespace std;

TEST(Transform,SO3){
    default_random_engine e;

    double pitch=uniform_real_distribution<double>(-M_PI/2,M_PI/2)(e);
    double yaw  =uniform_real_distribution<double>(-M_PI,M_PI)(e);
    double roll =uniform_real_distribution<double>(-M_PI/2,M_PI/2)(e);

    SO3 q=SO3::fromPitchYawRoll(pitch,yaw,roll);

    EXPECT_NEAR(pitch,q.getPitch(),1e-5);
    EXPECT_NEAR(yaw,q.getYaw(),1e-5);
    EXPECT_NEAR(roll,q.getRoll(),1e-5);

    // X: forward Y: right Z: down
    SO3 qRoll =SO3::exp(Point3d(roll,0,0));
    SO3 qPitch=SO3::exp(Point3d(0,pitch,0));
    SO3 qYaw  =SO3::exp(Point3d(0,0,yaw));

    SO3 q1=qYaw*qPitch*qRoll;
    EXPECT_EQ(q,q1);

    Matrix3d m=q.getMatrix();
    q1=SO3(m);
    EXPECT_EQ(q,q1);

    Point3d abc=q.log();
    q1=SO3::exp(abc);
    EXPECT_EQ(q,q1);
    EXPECT_EQ(SO3(),q.inverse()*q);

    uniform_real_distribution<double> pt_gen(-1000,1000);
    Point3d xyz(pt_gen(e),pt_gen(e),pt_gen(e));
    Point3d p1=q.inverse()*q*xyz;
    EXPECT_NEAR((xyz-p1).norm(),0,1e-6);
}


TEST(Transform,SE3){
    default_random_engine e;

    double pitch=uniform_real_distribution<double>(-M_PI/2,M_PI/2)(e);
    double yaw  =uniform_real_distribution<double>(-M_PI,M_PI)(e);
    double roll =uniform_real_distribution<double>(-M_PI/2,M_PI/2)(e);
    uniform_real_distribution<double> pt_gen(-1000,1000);

    SE3 T(SO3::fromPitchYawRoll(pitch,yaw,roll),
          Point3d(pt_gen(e),pt_gen(e),pt_gen(e)));

    Point3d pt(pt_gen(e),pt_gen(e),pt_gen(e));

    Point3d error=T*pt-T.get_translation()-T.get_rotation()*pt;
    EXPECT_NEAR(error.norm(),0,1e-8);

    SE3 T1=T.inverse()*T;
    EXPECT_NEAR(T1.get_translation().norm(),0,1e-8);
    EXPECT_EQ(T1.get_rotation(),SO3());

    Vector6d se3=T.log();
    SE3 T2=T.inverse()*SE3::exp(se3);
    EXPECT_NEAR(T2.get_translation().norm(),0,1e-8);
    EXPECT_EQ(T2.get_rotation(),SO3());
}

TEST(Transform,SIM3){
    default_random_engine e;

    double pitch=uniform_real_distribution<double>(-M_PI/2,M_PI/2)(e);
    double yaw  =uniform_real_distribution<double>(-M_PI,M_PI)(e);
    double roll =uniform_real_distribution<double>(-M_PI/2,M_PI/2)(e);
    uniform_real_distribution<double> pt_gen(-1000,1000);

    SIM3 S(SO3::fromPitchYawRoll(pitch,yaw,roll),
          Point3d(pt_gen(e),pt_gen(e),pt_gen(e)),
          100);
    SIM3 S1=SIM3::exp(S.log());
    EXPECT_EQ(S.get_rotation(),S1.get_rotation());
    EXPECT_NEAR((S.get_translation()-S1.get_translation()).norm(),0,1e-6);
    EXPECT_NEAR(S.get_scale(),S1.get_scale(),1e-6);

    SIM3 SS=S*S1;
    Point3d pt(pt_gen(e),pt_gen(e),pt_gen(e));

    Point3d diff=S*S1*pt-SS*pt;
    EXPECT_NEAR(diff.norm(),0,1e-10);

    SIM3 ST=S*S1.get_se3();
    EXPECT_EQ(S.get_scale(),ST.get_scale());

}
