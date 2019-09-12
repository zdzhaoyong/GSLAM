#include "PanelDisplays.h"

#include <regex>
#include "filesystem.hpp"

using namespace ghc::filesystem;
using namespace std;

namespace GSLAM{

void PanelDisplays::collectPlugins(){
    Svar config=_config;
    regex is_rviz_plugin("^(?:|lib)?qviz_([a-zA-Z\\d_]+).(?:|so|dll|dylib)$");
    for(auto folder:Registry::instance().paths()){
        if(!exists(path(folder))) continue;
        for(auto fileit:directory_iterator(folder))
        {
            smatch result;
            std::string filename = fileit.path().filename();
            if(std::regex_match(filename,result,is_rviz_plugin))
            {
                Svar var=Registry::load(filename);
                Svar qviz=var["gslam"]["displays"];
                if(!qviz.isObject()) continue;
                if(config["gslam"]["displays"].isObject())
                    config["gslam"]["displays"].as<SvarObject>().update(qviz);
                else
                    config["gslam"]["displays"]=qviz;
            }
        }
    }
    Svar displays=config["gslam"]["displays"];
    if(!displays.isObject()) return;
    displays["__name__"]="Display Plugins";
    tree->addDisplay(displays);

    std::map<std::string,Svar> plugins=displays.castAs<std::map<std::string,Svar>>();
    for(std::pair<std::string,Svar> plugin:plugins){
        if(!plugin.second.isObject()) continue;
        Svar create=plugin.second["__init__"];
        plugin.second["__menu__"]=Svar::object({{"Add",SvarFunction([create,this](){
                                                     this->tree->addDisplay(create());
                                                 })}});
        if(!create.isFunction()) return;
        tree->addDisplay(create());
    }
}

}
GSLAM_REGISTER_PANEL(displays,GSLAM::PanelDisplays);
