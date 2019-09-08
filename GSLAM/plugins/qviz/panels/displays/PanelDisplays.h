#pragma once

#include <QWidget>
#include <QDockWidget>
#include <QComboBox>
#include <QVBoxLayout>
#include <QPainter>
#include <QKeyEvent>
#include <GSLAM/core/GSLAM.h>

namespace GSLAM{

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
                setQImage(QImage(imageFile),true);
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

    bool setQImage(const QImage& qimage,bool flush=false)
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
        float scaleDelta=exp(e->delta()*0.001);
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
            GImage& img(gimage);
            GImage dst(img.rows,img.cols-(img.cols%4),img.type());
            int srcLineStep=img.elemSize()*img.cols;
            int dstLineStep=dst.elemSize()*dst.cols;
            for(int i=0;i<img.rows;i++)
                memcpy(dst.data+dstLineStep*i,img.data+srcLineStep*i,dstLineStep);
            gimage=dst;
        }

        _curGImage=gimage;
        if(gimage.type()==GSLAM::GImageType<uchar,3>::Type)
        {
            QImage qimage(gimage.data,gimage.cols,gimage.rows,QImage::Format_RGB888);
            return QImageWidget::setQImage(qimage,flush);
        }
        else if(gimage.type()==GSLAM::GImageType<uchar,4>::Type)
        {
            QImage qimage(gimage.data,gimage.cols,gimage.rows,QImage::Format_RGB32);
            return QImageWidget::setQImage(qimage,flush);
        }
        else if(gimage.type()==GSLAM::GImageType<uchar,1>::Type)
        {
            QImage qimage(gimage.data,gimage.cols,gimage.rows,QImage::Format_Indexed8);
            return QImageWidget::setQImage(qimage,flush);
        }
        // don't support other format yet
        return false;
    }

    GImage _curGImage;
};

class PanelDisplays: public QWidget
{
    Q_OBJECT
public:
    PanelDisplays(QWidget* parent,Svar config):
        QWidget(parent), _comb(NULL), _showMat(NULL)
    {
        _comb=new QComboBox(this);
        _showMat=new GImageWidget(this);
        QVBoxLayout *verticalLayout = new QVBoxLayout(this);

        verticalLayout->addWidget(_comb);
        verticalLayout->addWidget(_showMat);//,0,Qt::AlignCenter
        setLayout(verticalLayout);

        connect(_comb,SIGNAL(activated(QString)),
                this,SLOT(combActivated(QString)));
    }

    bool imshow(const std::string& name,const GSLAM::GImage& img)
    {
        if(!_var.exist(name))
        {
            _comb->addItem(QString::fromLocal8Bit(name.c_str()));
            _comb->setEditText(QString::fromLocal8Bit(name.c_str()));

            _var.set<GImage>(name,img);
            return _showMat->setImage(img);
        }
        _var.set<GImage>(name,img);

        QByteArray baCurrentText = _comb->currentText().toLocal8Bit();
        if(name == baCurrentText.data() && img.data)
        {
            return _showMat->setImage(img);
        }
        return true;
    }
protected slots:
    void combActivated(const QString& name)
    {
        _comb->setEditText(name);
        _showMat->setImage(_var.Get<GSLAM::GImage>(name.toStdString()));
    }

private:
    QComboBox*      _comb;
    GImageWidget*   _showMat;
    Svar            _var;
};

}
