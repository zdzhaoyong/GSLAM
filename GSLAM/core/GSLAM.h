#ifndef GSLAM_CORE_H
#define GSLAM_CORE_H

#define GSLAM_VERSION_MAJOR 3
#define GSLAM_VERSION_MINOR 0
#define GSLAM_VERSION_PATCH 0
#define GSLAM_VERSION (GSLAM_VERSION_MAJOR<<6|GSLAM_VERSION_MINOR<<4|GSLAM_VERSION_PATCH)
#define GSLAM_COMMAND_STRHELPER(COMMAND) #COMMAND
#define GSLAM_COMMAND_STR(COMMAND) GSLAM_COMMAND_STRHELPER(COMMAND)
#define GSLAM_VERSION_STR (GSLAM_COMMAND_STR(GSLAM_VERSION_MAJOR) "." \
                           GSLAM_COMMAND_STR(GSLAM_VERSION_MINOR) "." \
                           GSLAM_COMMAND_STR(GSLAM_VERSION_PATCH))

#define GSLAM_REGISTER_GLOG_SINKS \
    svar["gslam"]["setGlobalLogSinks"]=GSLAM::Svar::lambda(\
    [](std::shared_ptr<std::set<GSLAM::LogSink *> > sinks){\
        GSLAM::getLogSinksGlobal()=sinks;\
    });

#define GSLAM_REGISTER_MESSENGER \
    svar["gslam"]["setGlobalMessenger"]=GSLAM::Svar::lambda(\
    [](GSLAM::Messenger msg){\
        GSLAM::Messenger::instance()=msg;\
    });

#define GSLAM_REGISTER_APPLICATION(NAME,RUN) \
    EXPORT_SVAR_INSTANCE \
    REGISTER_SVAR_MODULE(NAME) \
{\
    GSLAM_REGISTER_GLOG_SINKS \
    GSLAM_REGISTER_MESSENGER \
    svar["gslam"]["apps"][#NAME]=GSLAM::SvarFunction(RUN);\
}

#define GSLAM_REGISTER_DATASET(D,E) \
    EXPORT_SVAR_INSTANCE \
    REGISTER_SVAR_MODULE(E){\
    GSLAM_REGISTER_GLOG_SINKS \
    GSLAM_REGISTER_MESSENGER \
    svar["gslam"]["datasets"][#E]=GSLAM::Svar::lambda([](){return (GSLAM::Dataset*)new D();});\
}

#define GSLAM_REGISTER_PANEL(NAME,CLS) \
    EXPORT_SVAR_INSTANCE \
REGISTER_SVAR_MODULE(NAME){\
GSLAM_REGISTER_GLOG_SINKS \
GSLAM_REGISTER_MESSENGER \
    svar["gslam"]["panels"][#NAME]=GSLAM::Svar::lambda([](QWidget* parent,GSLAM::Svar config){\
    return (QWidget*)new CLS(parent,config);});\
}

// System Basic
#include "Svar.h"
#include "Messenger.h"
#include "Glog.h"
#include "Timer.h"
#include "Registry.h"
#include "FileResource.h"

// Basic SLAM Objects
#include "Matrix.h"
#include "Point.h"
#include "SO3.h"
#include "SE3.h"
#include "SIM3.h"
#include "GImage.h"
#include "Camera.h"
#include "Map.h"

// Tools
#include "Undistorter.h"
#include "Dataset.h"
#include "Vocabulary.h"

//#include "GPS.h"
//#include "VideoFrame.h"
//#include "HashMap.h"

//#include "TileProjection.h"
//#include "TileManager.h"

#endif
