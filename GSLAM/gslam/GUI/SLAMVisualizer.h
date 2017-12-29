#ifndef SLAMVISUALIZER_H
#define SLAMVISUALIZER_H

#include <QWidget>
#include <qglviewer.h>

#include "../../core/GSLAM.h"

namespace GSLAM{

class SLAMVisualizerImpl;
class SLAMVisualizer : public QGLViewer, public GObjectHandle
{
public:
    SLAMVisualizer(QWidget* parent,QString pluginPath);

    SLAMPtr slam();

    virtual void draw();
    virtual void handle(const SPtr<GObject>& obj);

protected:
    SPtr<SLAMVisualizerImpl>   impl;
};

}
#endif
