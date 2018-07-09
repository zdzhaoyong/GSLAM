#ifndef SLAMVISUALIZER_H
#define SLAMVISUALIZER_H

#include <QWidget>

#include "GSLAM/core/GSLAM.h"
#include "GSLAM/core/Event.h"

#include "MapVisualizer.h"

namespace GSLAM{

class SLAMVisualizer : public QObject, public GObjectHandle
{
    Q_OBJECT
public:
    SLAMVisualizer(SLAMPtr slam_ptr,GObjectHandle* handle);

    SLAMPtr slam(){return _slam;}
    void releaseSLAM(){_slam=SLAMPtr();}

    virtual void draw();
    virtual void handle(const SPtr<GObject>& obj);

signals:
    void signalUpdate();
    void signalSetSceneRadius(qreal radius);
    void signalSetSceneCenter(qreal x,qreal y,qreal z);
    void signalSetViewPoint(qreal x,qreal y,qreal z,
                            qreal rw,qreal rx,qreal ry,qreal rz);
protected:
    void updateGL(){emit signalUpdate();}

    SLAMPtr                           _slam;
    GObjectHandle*                    _handle;
    std::string                       _name;

    std::map<std::string,SPtr<DrawableEvent> > _objects;
    MapPtr                            _map;
    SPtr<MapVisualizer>               _vis;
};

typedef SPtr<SLAMVisualizer> SLAMVisualizerPtr;
}
#endif
