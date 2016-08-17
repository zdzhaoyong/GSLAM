#ifndef SYSTEM_H
#define SYSTEM_H
#include <base/Thread/Thread.h>
#include <base/ClassLoader/ClassLoader.h>
#include <gui/gl/SignalHandle.h>
#include <GSLAM/core/GSLAM.h>
#include "MainWindow.h"

class System : public pi::Thread, pi::gl::EventHandle, pi::gl::Draw_Opengl
{
public:
    System();
    virtual void run();
    void SLAMMain();
    void SLAMDebug();

    virtual void Draw_Something();
private:
    SPtr<MainWindow>  _mainwindow;
    SPtr<GSLAM::SLAM> _slam;

};

#endif // SYSTEM_H
