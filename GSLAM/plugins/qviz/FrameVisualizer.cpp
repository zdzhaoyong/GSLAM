#ifdef HAS_QT
#include "FrameVisualizer.h"
#include <GSLAM/core/Timer.h>
#include <QPainter>
#include <QWheelEvent>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QDateTime>
using namespace std;

namespace GSLAM
{



InfomationViewer::InfomationViewer(QWidget* parent)
    :QTableWidget(parent),lastUpdateTime(0){
    setColumnCount(2);
    setHorizontalHeaderLabels({"name","value"});

    QHeaderView *HorzHdr = horizontalHeader();
#if QT_VERSION>=0x050000
    HorzHdr->setSectionResizeMode(QHeaderView::Stretch);
#else
    HorzHdr->setResizeMode(QHeaderView::Stretch);
#endif
}

QTableWidgetItem* InfomationViewer::setValue(int row,int col,QString val)
{
    if(item(row,col)!=NULL)
    {
        item(row,col)->setText(val);
        return item(row,col);
    }
    else
    {
        QTableWidgetItem* item=new QTableWidgetItem();
        item->setText(val);
        setItem(row,col,item);
        return item;
    }
}

QTableWidgetItem* InfomationViewer::setValue(int row,int col,double  val)
{
    if(item(row,col)!=NULL)
    {
        item(row,col)->setText(QString("%1").arg(val,0,'g',13));
        return item(row,col);
    }
    else
    {
        QTableWidgetItem* item=new QTableWidgetItem();
        item->setText(QString("%1").arg(val,0,'g',13));
        setItem(row,col,item);
        return item;
    }
}

void InfomationViewer::update(const FramePtr& frame,bool flush)
{
    double curTime=GSLAM::TicToc::timestamp();
    if(!flush&&curTime-lastUpdateTime<0.033) return;
    lastUpdateTime=curTime;

    vars["id"]=QString("%1").arg(frame->id());
    vars["type"]=frame->type().c_str();
    QDateTime tm=QDateTime::fromMSecsSinceEpoch(frame->timestamp()*1e3);
    vars["time"]=tm.toString("yyyy.MM.dd-hh:mm:ss.zzz");

    if(frame->cameraNum())
    {
        vars["cameraNum"]=QString("%1").arg(frame->cameraNum());
        for(int i=0;i<frame->cameraNum();i++)
        {
            vars[QString("Camera%1.Channels").arg(i)]=QString("%1").arg(frame->channelString(i).c_str());
            vars[QString("Camera%1.Info").arg(i)]=frame->getCamera(i).info().c_str();
        }
    }

    if(frame->getIMUNum())
    {
        vars["IMUNum"]=QString("%1").arg(frame->getIMUNum());
        Point3d pt;
        for(int i=0;i<frame->getIMUNum();i++)
        {
            if(frame->getPitchYawRoll(pt,i))
            {
                vars[QString("IMU%1.PitchYawRoll").arg(i)]=QString("%1,%2,%3")
                        .arg(pt.x,0,'g',13).arg(pt.y,0,'g',13).arg(pt.z,0,'g',13);
            }
            if(frame->getPYRSigma(pt,i))
            {
                vars[QString("IMU%1.PYRSigma").arg(i)]=QString("%1,%2,%3")
                        .arg(pt.x,0,'g',13).arg(pt.y,0,'g',13).arg(pt.z,0,'g',13);
            }
        }
    }

    if(frame->getGPSNum())
    {
        vars["GPSNum"]=QString("%1").arg(frame->getGPSNum());
        for(int i=0;i<frame->getGPSNum();i++)
        {
            Point3d pt;
            if(frame->getGPSLLA(pt,i))
            {
                vars[QString("GPS%1.LonLatAlt").arg(i)]=QString("%1,%2,%3")
                        .arg(pt.x,0,'g',13).arg(pt.y,0,'g',13).arg(pt.z,0,'g',13);
            }
            if(frame->getGPSLLASigma(pt,i))
            {
                vars[QString("GPS%1.LonLatAltSigma").arg(i)]=QString("%1,%2,%3")
                        .arg(pt.x,0,'g',13).arg(pt.y,0,'g',13).arg(pt.z,0,'g',13);
            }
        }
    }

    setRowCount(vars.size());
    int i=0;
    for(auto it:vars)
    {
        setValue(i,0,it.first)->setFlags(Qt::ItemIsEditable);
        setValue(i,1,it.second)->setFlags(Qt::ItemIsEditable);
        i++;
    }
}

void FrameVisualizer::slotFrameUpdated()
{
    GSLAM::ReadMutex lock(_mutex);
    if(!_curFrame) return;
    _infos->update(_curFrame);
    if(!_lastImageFrame) return;
//    GSLAM::ScopedTimer tm("FrameVisualizer::slotFrameUpdated");
    for(int camIdx=0;camIdx<_lastImageFrame->cameraNum();camIdx++)
    {
        GSLAM::FramePtr _curFrame=_lastImageFrame;
        if(_images.size()==camIdx)
        {
            _images.push_back(new GImageWidget(_splitter));
            _splitter->addWidget(_images[camIdx]);
        }
        GImageWidget* gimageW=_images[camIdx];
        int channelFlags=_curFrame->imageChannels();
        GImage img=_curFrame->getImage(camIdx);
        if(img.empty()) continue;

        if(img.cols%4!=0)
        {
#if defined(HAS_OPENCV)&&0
            cv::Mat src=img,dst(src.rows,src.cols-(img.cols%4),src.type());
            src(cv::Rect(0,0,dst.cols,dst.rows)).copyTo(dst);
            img=dst;
#else
            GImage dst(img.rows,img.cols-(img.cols%4),img.type());
            int srcLineStep=img.elemSize()*img.cols;
            int dstLineStep=dst.elemSize()*dst.cols;
            for(int i=0;i<img.rows;i++)
                memcpy(dst.data+dstLineStep*i,img.data+srcLineStep*i,dstLineStep);
            img=dst;
#endif
        }

        if((channelFlags&IMAGE_BGRA)&&img.channels()==3)
        {
            gimageW->setQImage(QImage(img.data,img.cols,img.rows,QImage::Format_RGB888).rgbSwapped(),true);
        }
        else if((channelFlags&IMAGE_RGBA)&&img.channels()==4)
        {
            gimageW->setQImage(QImage(img.data,img.cols,img.rows,QImage::Format_RGB32).rgbSwapped(),true);
        }
        else
        {
            gimageW->setImage(img,true);
        }
    }
    _lastImageFrame=FramePtr();
}

}
#endif
