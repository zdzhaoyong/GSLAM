#include "GSLAM/core/GSLAM.h"
#include "gtest.h"

using namespace GSLAM;

int run(Svar var){
    svar=var;
    if(var.get<bool>("help",false)){
        var.help();
        return 0;
    }
    testing::InitGoogleTest(&svar.Get<int>("argc"),(char**)svar.get<char**>("argv",nullptr));
    return RUN_ALL_TESTS();
}

GSLAM_REGISTER_APPLICATION(tests,run);
