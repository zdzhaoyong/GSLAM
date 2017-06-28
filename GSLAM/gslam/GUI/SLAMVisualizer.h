#ifndef SLAMVISUALIZER_H
#define SLAMVISUALIZER_H

#include <QWidget>
#include <qglviewer.h>

#include "../../core/GSLAM.h"

namespace GSLAM{

class SLAMVisualizer : public QGLViewer, public GObjectHandle
{
public:
    SLAMVisualizer(QWidget* parent,QString pluginPath)
        :QGLViewer(parent)
    {
        open(pluginPath);
    }

    bool open(QString pluginPath){
        _slam=SLAM::create(pluginPath.toStdString());
        if(_slam) _slam->setCallback(this);
        return _slam.get();
    }

    SLAMPtr& slam(){return _slam;}

    virtual void draw(){
        if(!_slam) return;
        if(_slam->isDrawable()) _slam->draw();
        else drawSLAM();
    }

    virtual void drawSLAM();

    virtual void handle(const SPtr<GObject>& obj);
protected:
    SLAMPtr   _slam;
};

}
#endif
