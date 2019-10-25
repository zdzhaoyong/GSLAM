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
        if(plugin.second.exist("__stay__"))
            tree->addDisplay(create());
    }
}

ObjectPropertyItem::ObjectPropertyItem(PropertyItem* parent,QTreeWidget* tree,QString name,Svar value,Svar updateFunc)
    : PropertyItem(parent,tree,name,value,updateFunc){
    if(_value.exist("__cbk__"))
        _updateFunc=value["__cbk__"];
    if(_value.exist("__name__"))
        setText(0,_value["__name__"].castAs<std::string>().c_str());

    for(auto child:_value.as<SvarObject>()._var){
        const std::string& name=child.first;
        if(name.empty()||name.front()=='_') continue;
        if(name=="visible"&&_value["visible"].is<bool>()){
            bool& visible=_value["visible"].as<bool>();
            if(visible)
                setIcon(1,QIcon(":/icon/visiable.png"));
            else
                setIcon(1,QIcon(":/icon/nVisiable.png"));
            continue;
        }
        Svar display=child.second;
        Svar callback=_value["__cbk__"+name];
        if     (display.isObject()) new ObjectPropertyItem(this,tree,name.c_str(),display,callback);
        else if(display.isArray())  new ArrayPropertyItem(this,tree,name.c_str(),display,callback);
        else if(display.is<bool>())  new BoolPropertyItem(this,tree,name.c_str(),display,callback);
        else if(display.is<Topic>())  new TopicPropertyItem(this,tree,name.c_str(),display,callback);
        else if(display.is<Point3ub>()) new ColorPropertyItem(this,tree,name.c_str(),display,callback);
        else  new JsonPropertyItem(this,tree,name.c_str(),display,callback);
    }
}
}
GSLAM_REGISTER_PANEL(displays,GSLAM::PanelDisplays);
