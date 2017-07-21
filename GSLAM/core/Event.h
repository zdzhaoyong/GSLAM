#ifndef GSLAM_EVENT_H
#define GSLAM_EVENT_H
#include "GSLAM.h"

namespace GSLAM {
#define REG_EVENT(EVENTNAME,DATAFORMAT,DATANAME)\
    class EVENTNAME : public GEvent{\
    public:\
        virtual std::string type()const{return #EVENTNAME;}\
        EVENTNAME (const DATAFORMAT& DATANAME):_##DATANAME(DATANAME){}\
DATAFORMAT _##DATANAME;}

#define REG_EVENT2(E,D1,N1,D2,N2)\
    class E : public GEvent{public:\
        virtual std::string type()const{return #E;}\
        E (const D1& N1,const D2& N2):_##N1(N1),_##N2(N2){}\
    D1 _##N1;D2 _##N2;}

class GEvent: public GObject
{
public:
    GEvent():_handled(false),_time(0.){}

    virtual std::string type()const{return "GEvent";}

    /** Set whether this event has been handled by an event handler or not.*/
    void setHandled(bool handled=true) const { _handled = handled; }

    /** Get whether this event has been handled by an event handler or not.*/
    bool getHandled() const { return _handled; }


    /** set time in seconds of event. */
    void setTime(double time) { _time = time; }

    /** get time in seconds of event. */
    double getTime() const { return _time; }

protected:
    virtual ~GEvent() {}
    mutable bool        _handled;
    double              _time;
};

REG_EVENT(CommandEvent,std::string,cmd);
REG_EVENT(ScenceCenterEvent,Point3d,center);
REG_EVENT(ScenceRadiusEvent,double,radius);
REG_EVENT(SetViewPoseEvent,SE3,pose);
REG_EVENT(CurrentFrameEvent,FramePtr,frame);
REG_EVENT2(DebugImageEvent,GImage,img,std::string,name);


}
#endif

