#if defined(HAS_QT)


#include "SLAMVisualizer.h"

namespace GSLAM{

SLAMVisualizer::SLAMVisualizer(SLAMPtr slam_ptr,GObjectHandle* handle)
    : _slam(slam_ptr),_handle(handle)
{
    _name=_slam->type();
    svar.call("AddLayer",_name);

    _slam->setSvar(svar);
    _slam->setCallback(this);
}

void SLAMVisualizer::draw(){
    if(!svar.GetInt(_name)) return;
    if(_slam&&_slam->isDrawable()) _slam->draw();
    if(_vis)
        _vis->draw();
    for(auto& d:_objects)
        if(svar.GetInt(d.first))
            d.second->_obj->draw();
}

void SLAMVisualizer::handle(const std::shared_ptr<GObject>& obj){
    if(!obj) return;
    if(auto e=std::dynamic_pointer_cast<Map>(obj))
    {
        if(_map!=e){
            _map=e;
            if(!_vis)
                _vis=std::shared_ptr<MapVisualizer>(new MapVisualizer(_map,_name,dynamic_cast<GObjectHandle*>(this)));
            else _vis->setMap(_map);
        }
        _vis->update();

        updateGL();
//        LOG(INFO)<<"Map";
    }
    else if(FramePtr e=std::dynamic_pointer_cast<MapFrame>(obj))
    {
        if(svar.GetInt("FreeKeyFrameImage",1))
            e->setImage(GImage());
        if(_vis)
            _vis->update();
        updateGL();
//        LOG(INFO)<<"MapFrame";
    }
    else if(auto e=std::dynamic_pointer_cast<CurrentFrameEvent>(obj))
    {
        if(_vis)
            _vis->updateCurframe(e->_frame);
        updateGL();
//        LOG(INFO)<<"CurrentFrameEvent";
    }
    else if(auto e=std::dynamic_pointer_cast<ScenceCenterEvent>(obj))
    {
        Point3d center=e->_center;
        emit signalSetSceneCenter(center.x,center.y,center.z);
    }
    else if(auto e=std::dynamic_pointer_cast<ScenceRadiusEvent>(obj))
    {
        emit signalSetSceneRadius(e->_radius);
    }
    else if(auto e=std::dynamic_pointer_cast<SetViewPoseEvent>(obj)){
        SE3 pose=e->_pose;
        const Point3f& t=pose.get_translation();
        const SO3f& r=pose.get_rotation();
        emit signalSetViewPoint(t.x,t.y,t.z,r.w,r.x,r.y,r.z);
//        LOG(INFO)<<"signalSetViewPoint";
    }
    else if(auto e=std::dynamic_pointer_cast<DrawableEvent>(obj)){
        _objects[_name+"."+e->_name]=e;
        svar.call("AddLayer",_name+"."+e->_name);
//        LOG(INFO)<<"DrawableEvent";
        updateGL();
    }
    else if(_handle) return _handle->handle(obj);
}

}
#endif
