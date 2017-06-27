#ifndef FRAMEVISUALIZER_H
#define FRAMEVISUALIZER_H
#include "../../core/GSLAM.h"
#include "../../core/Svar.h"
#include <QWidget>
#include <QVBoxLayout>
#include <QSplitter>
#include <QTableWidget>

namespace GSLAM
{
class GImageWidget;

class InfomationViewer : public QTableWidget
{
    Q_OBJECT
public:
    InfomationViewer(QWidget* parent);
    QTableWidgetItem* setValue(int row,int col,QString val);
    QTableWidgetItem* setValue(int row,int col,double  val);
    void              update(const FramePtr& frame);
};

class FrameVisualizer: public QWidget
{
    Q_OBJECT
public:
    FrameVisualizer(QWidget* parent=NULL):QWidget(parent){
        _splitter=new QSplitter(Qt::Vertical,this);
        _imageLayout=new QVBoxLayout(this);
        _imageLayout->addWidget(_splitter);
        _infos=new InfomationViewer(_splitter);
        connect(this,SIGNAL(signalFrameUpdated()),this,SLOT(slotFrameUpdated()));
    }
    virtual ~FrameVisualizer(){}

    typedef SPtr<FrameVisualizer> FrameVisualizerPtr;
    static SvarWithType<FrameVisualizerPtr>& visualizers(){
        static SvarWithType<FrameVisualizerPtr> globalVis;
        return globalVis;
    }

    void setFrame(const FramePtr& frame){
        GSLAM::WriteMutex lock(_mutex);
        _curFrame=frame;
        emit signalFrameUpdated();
    }
signals:
    void signalFrameUpdated();
public slots:
    void slotFrameUpdated();

protected:
    GSLAM::MutexRW              _mutex;
    FramePtr                    _curFrame;
    std::vector<GImageWidget*>  _images;
    InfomationViewer*           _infos;
    QVBoxLayout*                _imageLayout;
    QSplitter*                  _splitter;
};

}

#endif
