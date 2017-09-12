#if defined(HAS_QT)&&defined(HAS_QGLVIEWER)
#include "SLAMVisualizer.h"
#include "../../core/Event.h"

namespace GSLAM{

void SLAMVisualizer::drawSLAM(){}


void SLAMVisualizer::handle(const SPtr<GObject>& obj){
    if(!obj) return;
    if(obj->type()=="ScenceCenterEvent")
    {
        ScenceCenterEvent* sce=((ScenceCenterEvent*)obj.get());
        Point3d center=sce->_center;
        setSceneCenter(qglviewer::Vec(center.x,center.y,center.z));
        sce->setHandled();
        update();
    }
    else if(obj->type()=="ScenceRadiusEvent")
    {
        ScenceRadiusEvent* radiusE=(ScenceRadiusEvent*)obj.get();
        setSceneRadius(radiusE->_radius);
        update();
    }
}

}
#endif
