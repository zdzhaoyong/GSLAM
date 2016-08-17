#include "System.h"
#include <base/Svar/Svar.h>
#include <base/Types/VecParament.h>
#include <base/Utils/TestCase.h>
#include <base/Time/DateTime.h>
#include <set>
#include <GSLAM/core/GSLAM.h>
#include "VideoReader.h"

#ifdef DEBUG_WithQT
#include <GSLAM/orbslam/ORBSLAM.h>
#endif

using namespace std;

int excuteTest()
{
    std::cerr<<"Started test at "<<pi::DateTime()<<std::endl;
    pi::TestCase test;
    // filt cases
    if(svar.exist("Cases")||svar.exist("CasesIgnore"))
    {
        VecParament<std::string> CasesVec=svar.get_var("Cases",VecParament<std::string>());
        VecParament<std::string> CasesIgnoreVec=svar.get_var("CasesIgnore",VecParament<std::string>());

        if(CasesVec.size())
        {
            std::set<std::string> CasesMap;
            CasesMap.insert(CasesVec.data.begin(),CasesVec.data.end());
            std::map<std::string,pi::TestCase*> data=test.cases.get_data();
            for(std::map<std::string,pi::TestCase*>::iterator it=data.begin();it!=data.end();it++)
            {
                if(!CasesMap.count(it->first)) test.cases.erase(it->first);
            }
        }

        for(size_t i=0;i<CasesIgnoreVec.size();i++) test.cases.erase(CasesIgnoreVec[i]);
    }
    else
    {
        std::cerr<<"\"Cases\" and \"CasesIgnore\" not setted, test all cases defaultly.\n";
    }

    test.callAll();
    std::cerr<<test.generateReport();
    return 0;
}


System::System()
{
    if(svar.GetInt("WithQt"))
    {
        _mainwindow=SPtr<MainWindow>(new MainWindow());
    }
}

void System::run()
{
    try{
        string act=svar.GetString("Act","");
        if("Tests"==act)
        {
            excuteTest();
        }
        else if("SLAM"==act)
        {
            SLAMMain();
        }
        else //if("SLAMDebug"==act)
        {
#ifdef DEBUG_WithQT
            SLAMDebug();
#else
            cout<<"'Act' not setted. Lauching tests...\n";
            excuteTest();
#endif
        }
    }
    catch(const std::exception& e)
    {
        cout<<"System abort since exception:"<<e.what()<<endl;
    }
}

void System::SLAMDebug()
{
#ifdef DEBUG_WithQT
    string videoName=svar.GetString("VideoReader","VideoReader");
    VideoReader video(videoName);
    if(!video.isOpened())
    {
        cout<<"Failed to open video "+videoName<<endl;return;
    }

    _slam=SPtr<GSLAM::SLAM>(new GSLAM::ORBSLAM());
    cout<<"Loaded SLAM system "<<_slam->type()<<endl;

    if(!_slam->valid()) {cerr<<_slam->type()<<" is not valid!\n"; return;}

    if(_mainwindow.get())
    {
        _mainwindow->call("Show");
        if(_mainwindow->getWin3D())
        {
            _mainwindow->getWin3D()->SetDraw_Opengl(this);
            _mainwindow->getWin3D()->SetEventHandle(this);
        }
    }

    int& pause=svar.GetInt("Pause");
    while (!shouldStop()) {
        if(pause) {sleep(10);continue;}
        GSLAM::FramePtr frame=video.grabFrame();
        if(!frame.get())
        {
            cerr<<"Video finished.\n";return;
        }
        if(_slam->track(frame))
        {
            cout<<frame->type()<<":"<<frame->_timestamp<<","<<frame->getPose()<<endl;

            if(_mainwindow.get())
                _mainwindow->call("Update");
        }
    }

    _mainwindow.reset();
    _slam.reset();
    cout<<"SLAM system stoped.\n";cout.flush();
#else
    SLAMMain();
#endif
}

void System::SLAMMain()
{
    string videoName=svar.GetString("VideoReader","VideoReader");
    VideoReader video(videoName);
    if(!video.isOpened())
    {
        cout<<"Failed to open video "+videoName<<endl;return;
    }

    std::string slamLibraryPath=svar.GetString("SLAM.LibraryPath","../libs/libgslam_orbslam.so");
    pi::ClassLoader<GSLAM::SLAM> cl;
    cl.loadLibrary(slamLibraryPath);
    if(!cl.isLibraryLoaded(slamLibraryPath))
    {
        cerr<<"Failed to load library "<<slamLibraryPath<<endl;
    }
    else
    {
        cout<<cl<<endl;
    }
    _slam=SPtr<GSLAM::SLAM>(cl.create("GSLAM::ORBSLAM"));
    cout<<"Loaded SLAM system "<<_slam->type()<<endl;

    if(!_slam->valid()) {cerr<<_slam->type()<<" is not valid!\n"; return;}

    if(_mainwindow.get())
    {
        _mainwindow->call("Show");
        if(_mainwindow->getWin3D())
        {
            _mainwindow->getWin3D()->SetDraw_Opengl(this);
            _mainwindow->getWin3D()->SetEventHandle(this);
        }
    }

    int& pause=svar.GetInt("Pause");
    while (!shouldStop()) {
        if(pause) {sleep(10);continue;}
        GSLAM::FramePtr frame=video.grabFrame();
        if(!frame.get())
        {
            cerr<<"Video finished.\n";return;
        }
        if(_slam->track(frame))
        {
            cout<<frame->type()<<":"<<frame->_timestamp<<","<<frame->getPose()<<endl;

            if(_mainwindow.get())
                _mainwindow->call("Update");
        }
    }

    _mainwindow.reset();
    _slam.reset();
    cout<<"SLAM system stoped.\n";cout.flush();
}

void System::Draw_Something()
{
    if(_slam.get()) _slam->draw();
}
