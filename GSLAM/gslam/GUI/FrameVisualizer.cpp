#ifdef HAS_QT
#include "FrameVisualizer.h"

#include <QPainter>
#include <QWheelEvent>
#include <QVBoxLayout>
#include <QHeaderView>
#ifdef HAS_OPENCV
#include <opencv2/imgproc/imgproc.hpp>
#endif
using namespace std;

namespace GSLAM
{

class QImageWidget : public QWidget
{
public:
    QImageWidget(QWidget* parent)
        :imageUpdated(false), QWidget(parent){
        setBaseSize(QSize(100,100));
        setMinimumHeight(128);

        mStartedMove=false;
    }

    void paintEvent(QPaintEvent* e)
    {
        if(imageUpdated)
        {
            if(!curImage.isNull())
            {
                curPixmap=QPixmap::fromImage(curImage);
                mouseDoubleClickEvent(NULL);
                curImage=QImage();
            }
            else
            {
                setImage(QImage(imageFile),true);
            }
            imageUpdated=false;
        }
        if(curPixmap.isNull()) return;
        QPainter painter(this);
        QRectF srcRect(0, 0, curPixmap.width(), curPixmap.height());
        painter.drawPixmap(tgtRect,curPixmap,srcRect);
    }

    virtual void resizeEvent(QResizeEvent* e)
    {
        mouseDoubleClickEvent(NULL);
    }

    bool setImage(const QImage& qimage,bool flush=false)
    {
        if(qimage.isNull()) return false;

        if(flush)
        {
            curPixmap=QPixmap::fromImage(qimage);
            mouseDoubleClickEvent(NULL);
        }
        else
        {
            curImage=qimage;
        }
        imageUpdated=true;
        update();
        return true;
    }

    bool setImage(const QString& imgFile)
    {
        imageFile=imgFile;
        imageUpdated=true;
        update();
        return true;
    }

    virtual void wheelEvent(QWheelEvent *e)
    {
        float scale=std::max(tgtRect.width()/(float)width(),tgtRect.height()/(float)height());
        float scaleDelta=std::exp(e->delta()*0.001);
        scale=scaleDelta*scale;

        QPointF newDxy=QPointF(e->x(),e->y())-QPointF(e->x()-tgtRect.x(),e->y()-tgtRect.y())*scaleDelta;
        tgtRect=QRectF(newDxy.x(),newDxy.y(),tgtRect.width()*scaleDelta,tgtRect.height()*scaleDelta);

        if(scale<1) mouseDoubleClickEvent(NULL);
        update();
    }

    virtual void mousePressEvent(QMouseEvent* e)
    {
        mStartedMove=true;
        mStartPosition=e->pos();
    }

    virtual void mouseMoveEvent(QMouseEvent* e)
    {
        if(mStartedMove)
        {
            QPoint mousePoint = e->pos();

            int y_offset = mousePoint.y() - mStartPosition.y();
            int x_offset = mousePoint.x() - mStartPosition.x();
            tgtRect=QRectF(tgtRect.x()+x_offset,tgtRect.y()+y_offset,tgtRect.width(),tgtRect.height());
            update();
            mStartPosition=mousePoint;
        }
    }

    virtual void mouseReleaseEvent(QMouseEvent* e)
    {
        mStartedMove=false;
    }

    virtual void mouseDoubleClickEvent(QMouseEvent *e)
    {
        float ww = this->width(), wh = this->height();
        float iw = curPixmap.width(), ih = curPixmap.height();
        float dx=0,dy=0;

        if(ww/wh > iw/ih)
        {
            ww = iw*wh/ih;
            dx= (this->width()-ww)/2;
        }
        else
        {
            wh = ih*ww/iw;
            dy = (this->height()-wh)/2;
        }

        tgtRect=QRectF(dx, dy, ww, wh);
        update();
    }

    virtual void keyPressEvent(QKeyEvent* e)
    {
        switch (e->key()) {
        case Qt::Key_Escape:
            setWindowFlags(Qt::SubWindow);
            showNormal();
            parentWidget()->show();
            parentWidget()->move(0,0);
            break;
        default:
            break;
        }
    }

    bool    mStartedMove;
    QPoint  mStartPosition;

    QRectF  tgtRect;
    QString imageFile;
    bool    imageUpdated;
    QImage  curImage;
    QPixmap curPixmap;
};


class GImageWidget : public QImageWidget
{
public:
    GImageWidget(QWidget *parent):QImageWidget(parent){}
    bool setImage(const GImage& ImageInput,bool flush=false)// flush can only called by GUI thread
    {
        GImage gimage=ImageInput;
        if(gimage.empty()) return false;
        if(gimage.cols%4!=0)
        {
#ifdef HAS_OPENCV
            cv::Mat src=gimage,dst(src.rows,src.cols-(gimage.cols%4),src.type());
            src(cv::Rect(0,0,dst.cols,dst.rows)).copyTo(dst);
            gimage=dst;
#endif
        }

        _curGImage=gimage;
        if(gimage.type()==GSLAM::GImageType<uchar,3>::Type)
        {
            QImage qimage(gimage.data,gimage.cols,gimage.rows,QImage::Format_RGB888);
            return QImageWidget::setImage(qimage,flush);
        }
        else if(gimage.type()==GSLAM::GImageType<uchar,4>::Type)
        {
            QImage qimage(gimage.data,gimage.cols,gimage.rows,QImage::Format_RGB32);
            return QImageWidget::setImage(qimage,flush);
        }
        else if(gimage.type()==GSLAM::GImageType<uchar,1>::Type)
        {
            QImage qimage(gimage.data,gimage.cols,gimage.rows,QImage::Format_Indexed8);
            return QImageWidget::setImage(qimage,flush);
        }
        // don't support other format yet
        return false;
    }

    GImage _curGImage;
};

InfomationViewer::InfomationViewer(QWidget* parent)
    :QTableWidget(parent){
    setColumnCount(2);
    setHorizontalHeaderLabels({"name","value"});

    QHeaderView *HorzHdr = horizontalHeader();
#if QT_VERSION>=0x050000
    HorzHdr->setSectionResizeMode(QHeaderView::Interactive);
#else
    HorzHdr->setResizeMode(QHeaderView::Interactive);
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

void InfomationViewer::update(const FramePtr& frame)
{
    vars["id"]=QString("%1").arg(frame->id());
    vars["type"]=frame->type().c_str();
    vars["timestamp"]=QString("%1").arg(frame->timestamp(),0,'g',13);

    if(frame->cameraNum())
    {
        vars["cameraNum"]=QString("%1").arg(frame->cameraNum());
        for(int i=0;i<frame->cameraNum();i++)
        {
            vars[QString("Camera%1.Channels").arg(i)]=QString("%1").arg(frame->imageChannels(i));
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
    if(!_curFrame) return;
    GSLAM::ReadMutex lock(_mutex);
    _infos->update(_curFrame);
    for(int camIdx=0;camIdx<_curFrame->cameraNum();camIdx++)
    {
        if(_images.size()==camIdx)
        {
            _images.push_back(new GImageWidget(_splitter));
            _splitter->addWidget(_images[camIdx]);
        }
        GImageWidget* gimageW=_images[camIdx];
        int channelFlags=_curFrame->imageChannels();
        auto img=_curFrame->getImage(camIdx);
        if((channelFlags&IMAGE_BGRA)&&img.channels()==3)
        {
#ifdef HAS_OPENCV
            cv::Mat img=_curFrame->getImage(camIdx,IMAGE_BGRA);
            if(img.empty()) return;
            cv::cvtColor(img,img,CV_BGR2RGB);
            gimageW->setImage(img,true);
#else
            gimageW->setImage(_curFrame->getImage(camIdx,IMAGE_BGRA),true);
#endif
        }
        else if((channelFlags&IMAGE_RGBA)&&img.channels()==4)
        {
#ifdef HAS_OPENCV
            cv::Mat img=_curFrame->getImage(camIdx,IMAGE_RGBA);
            if(img.empty()) return;
            cv::cvtColor(img,img,CV_BGRA2RGBA);
            gimageW->setImage(img,true);
#else
            gimageW->setImage(_curFrame->getImage(camIdx,IMAGE_RGBA),true);
#endif
        }
        else if(channelFlags&IMAGE_DEPTH)
        {
            gimageW->setImage(_curFrame->getImage(camIdx,IMAGE_DEPTH),true);
        }
        else
        {
            gimageW->setImage(img);
        }
    }
}

}
#endif
