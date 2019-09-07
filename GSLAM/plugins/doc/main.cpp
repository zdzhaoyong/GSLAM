#include <GSLAM/core/GSLAM.h>

using namespace GSLAM;

int run(Svar config){

    config.arg<std::string>("plugin","","The svar module plugin wanna to show.");
    config.arg<std::string>("key","","The name in the module wanna to show");

    if(config.get("help",false)) {return config.help();}

    std::string pluginPath=config.get<std::string>("plugin","");
    std::string key=config.get("key","");

    Svar inst=Registry::load(pluginPath);
    GSLAM::Svar var=key.empty()?(inst):inst.get(key,Svar(),true);

    if(var.isFunction())
        std::cout<<var.as<SvarFunction>()<<std::endl;
    else if(var.isClass())
        std::cout<<var.as<SvarClass>()<<std::endl;
    else std::cout<<var;
    return 0;
}

GSLAM_REGISTER_APPLICATION(doc,run);
