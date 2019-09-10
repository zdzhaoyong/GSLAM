#ifndef FRAMEVISUALIZER_H
#define FRAMEVISUALIZER_H
#include "GSLAM/core/GSLAM.h"
#include "GSLAM/core/Svar.h"
#include <GSLAM/core/Messenger.h>

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
    std::map<QString,QString>   vars;
};

class FrameVisualizer: public QWidget,public GObjectHandle
{
    Q_OBJECT
public:
    FrameVisualizer(QWidget* parent,Svar config)
        :QWidget(parent){
        _splitter=new QSplitter(Qt::Vertical,this);
        _imageLayout=new QVBoxLayout(this);
        _imageLayout->addWidget(_splitter);
        _infos=new InfomationViewer(this);
        _splitter->addWidget(_infos);
        connect(this,SIGNAL(signalFrameUpdated()),this,SLOT(slotFrameUpdated()));
        _subFrameVis=GSLAM::Messenger::instance().subscribe("dataset/frame",[this](FramePtr frame){
                setFrame(frame);
        });
    }
    virtual ~FrameVisualizer(){
        LOG(INFO)<<"Frame Visualizer released.";
    }

    void setFrame(FramePtr frame){
        {
            GSLAM::WriteMutex lock(_mutex);
            if(_curFrame)
                _curFrame->setImage(GSLAM::GImage());
            _curFrame=frame;
        }
        if(!frame) return;
        emit signalFrameUpdated();
    }

    FramePtr curFrame(){return _curFrame;}

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
    GSLAM::Subscriber           _subFrameVis;
};

}

#endif
