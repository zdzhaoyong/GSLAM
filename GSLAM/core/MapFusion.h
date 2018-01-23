#ifndef GSLAM_MAPFUSION_H
#define GSLAM_MAPFUSION_H

#include <GSLAM/core/GSLAM.h>

#define USE_MAPFUSION_PLUGIN(C) extern "C"{\
    GSLAM::MapFusionPtr createMapFusionInstance(){return GSLAM::MapFusionPtr(new C());}}

namespace GSLAM {

class MapFusion;
typedef SPtr<MapFusion> MapFusionPtr;
typedef GSLAM::MapFusionPtr (*funcCreateMapFusionInstance)();

class MapFusion : public GObject
{
public:
    MapFusion():cbk(NULL){}

    virtual std::string type()const{return "MapFusionNone";}

    virtual bool valid(){return false;}                         // whether if this Map2DFusion usable
    virtual void draw(){}                                       // Plot things with OpenGL
    virtual bool feed(FramePtr frame){return false;}
    virtual bool feed(MapPtr   map){return false;}
    virtual bool save(const std::string& filePath){return false;}

    virtual bool setCallBack(GObjectHandle* callback=NULL){cbk=callback;return true;}

    static MapFusionPtr create(const std::string& pluginName){
        SPtr<SharedLibrary> plugin=Registry::get(pluginName);
        if(!plugin) return MapFusionPtr();
        funcCreateMapFusionInstance createFunc=(funcCreateMapFusionInstance)plugin->getSymbol("createMapFusionInstance");
        if(!createFunc) return MapFusionPtr();
        else return createFunc();
    }
protected:
    GObjectHandle* cbk;
};


}

#endif
