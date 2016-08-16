#include "ORBSLAM.h"
#include <base/ClassLoader/ClassLibrary.h>

#include <iostream>
#include <fstream>

#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>

#include <base/Svar/Svar.h>
#include <base/Time/Timer.h>
#include <gui/gl/Win3D.h>

using namespace pi;
using namespace ORB_SLAM;
namespace GSLAM{


ORBSLAM::ORBSLAM()
{
    start();

}

void ORBSLAM::run()
{
    //Load ORB Vocabulary
    string strVocFile = svar.GetString("ORBVocabularyFile", "./Data/GSLAM/ORBvoc.yml");
    cout << endl << "Loading ORB Vocabulary: " << strVocFile << " ...";
    Vocabulary = SPtr<ORBVocabulary>(new ORBVocabulary());
    Vocabulary->loadFast(strVocFile);
    cout << endl;

    //Create KeyFrame Database
    Database = SPtr<KeyFrameDatabase>(new KeyFrameDatabase(*Vocabulary));

    //Create the map
    World = SPtr<ORB_SLAM::Map>(new ORB_SLAM::Map);

    //Initialize the Tracking Thread and launch
    Tracker = SPtr<Tracking>(new Tracking(Vocabulary.get(), World.get()));
    Tracker->SetKeyFrameDatabase(Database.get());

    //Initialize the Local Mapping Thread and launch
    LocalMapper = SPtr<LocalMapping>(new LocalMapping(World.get()));
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run, LocalMapper.get());

    //Initialize the Loop Closing Thread and launch
    LoopCloser = SPtr<LoopClosing>(new LoopClosing(World.get(), Database.get(), Vocabulary.get()));
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, LoopCloser.get());

    // running map cleanup thread
//    boost::thread mapClearnThread(&ORB_SLAM::Map::Run, World);

    //Set pointers between threads
    Tracker->SetLocalMapper(LocalMapper.get());
    Tracker->SetLoopClosing(LoopCloser.get());

    LocalMapper->SetTracker(Tracker.get());
    LocalMapper->SetLoopCloser(LoopCloser.get());

    LoopCloser->SetTracker(Tracker.get());
    LoopCloser->SetLocalMapper(LocalMapper.get());

    //This "main" thread will show the current processed frame and publish the map
    double fps = svar.GetDouble("Video.FPS", 60);
    if(fps == 0) fps = 30;

    pi::Rate r(fps);

    while ( !shouldStop() ) {
        r.sleep();
    }
}

bool ORBSLAM::track(FramePtr &frame)
{
    return Tracker->track(frame);
}

bool ORBSLAM::valid()const
{
    return Vocabulary.get()&&Database.get()&&LocalMapper.get()&&LoopCloser.get()&&World.get()&&Tracker.get();
}

void ORBSLAM::call(const std::string& command,void* arg)
{
    if(command=="Start")
    {
        this->start();
    }
    else if(command=="Stop")
    {
        this->stop();
    }
    else if(command=="IsRunning"&&arg)
    {
        bool* runningFlag=(bool*)arg;
        *runningFlag=isRunning();
    }

}

void ORBSLAM::draw()
{
    if(!valid()) return;
    Tracker->Draw_Something();
    World->Draw_Something();
}

}

PIL_BEGIN_MANIFEST(GSLAM::SLAM)
    PIL_EXPORT_CLASS(GSLAM::ORBSLAM)
PIL_END_MANIFEST


void PILInitializeLibrary()
{
    std::cout << "Initializing ORBSLAM library..." << std::endl;
}


void PILUninitializeLibrary()
{
    std::cout << "ORBSLAM library Uninitialized" << std::endl;
}

