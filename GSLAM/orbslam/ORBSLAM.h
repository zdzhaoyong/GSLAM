#ifndef ORBSLAM_H
#define ORBSLAM_H
#include <GSLAM/core/GSLAM.h>
#include "Map.h"
#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Converter.h"

namespace GSLAM{

class ORBSLAM : public GSLAM::SLAM, public pi::Thread
{
public:
    ORBSLAM();
    virtual ~ORBSLAM(){}
    virtual std::string type()const{return "ORBSLAM";}
    virtual bool valid()const;

    virtual bool track(FramePtr& frame);

    virtual void draw();

    virtual void call(const std::string& command,void* arg=NULL);

    virtual void run();

private:
    SPtr<ORB_SLAM::ORBVocabulary>           Vocabulary;
    SPtr<ORB_SLAM::KeyFrameDatabase>        Database;
    SPtr<ORB_SLAM::Map>                     World;
    SPtr<ORB_SLAM::Tracking>                Tracker;
    SPtr<ORB_SLAM::LocalMapping>            LocalMapper;
    SPtr<ORB_SLAM::LoopClosing>             LoopCloser;
};

}
#endif // ORBSLAM_H
