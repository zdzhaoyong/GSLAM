#include "gtest.h"
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
