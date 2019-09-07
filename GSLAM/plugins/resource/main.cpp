#include <GSLAM/core/GSLAM.h>

using namespace GSLAM;

int run(Svar config){
    std::string map=config.arg<std::string>("map","","eg. {\"name1.txt\":\"name1.txt\",\"2.txt\":\"2.txt\"}");
    std::string cpp=config.arg<std::string>("cpp","resource.cpp","the output resource cpp");

    if(config.get("help",false)) {
        config.help();
        return 0;
    }

    if(map.length()<=0){
        std::cerr<<"Please use -map to set mapping rules";
        return -1;
    }

    Svar mapVar;
    std::stringstream(map)>>mapVar;

    return GSLAM::FileResource::exportResourceFile(mapVar.castAs<std::map<std::string,std::string>>(),cpp)?0:-2;
}

GSLAM_REGISTER_APPLICATION(resource,run);
