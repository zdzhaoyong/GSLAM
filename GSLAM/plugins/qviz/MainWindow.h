#ifdef HAS_QT
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QAction>
#include <QTreeWidget>
#include <memory>
#include <GSLAM/core/GSLAM.h>

#include "FrameVisualizer.h"
#include "SLAMVisualizer.h"

#include "GSLAM/core/Svar.h"
#include "GSLAM/core/GSLAM.h"
#include "GSLAM/core/Dataset.h"
#include "GSLAM/core/Svar.h"
#include "GSLAM/core/Timer.h"
#include "GSLAM/core/VecParament.h"
#include <GSLAM/core/Messenger.h>

#include "QGLViewer/qglviewer.h"

namespace GSLAM{

class Win3D : public QGLViewer
{
public:
    Win3D(QWidget* parent)
        : QGLViewer(parent),_fastDraw(svar.GetInt("FastDrawing",0)){}

    virtual void draw()
    {
        _fastDraw=0;
        const auto& objects=_objects;
        for(auto it : objects)
            it.second->draw();
    }

    virtual void fastDraw()
    {
        _fastDraw=1;
        const auto& objects=_objects;
        for(auto it : objects)
            it.second->draw();
    }

    int&                            _fastDraw;
    std::map<std::string,GObjectPtr>        _objects;
};

class MainWindow: public QMainWindow,GObjectHandle
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow(){
        slotStop();
    }

    virtual int setupLayout(void);
    virtual void handle(const std::shared_ptr<GObject>& obj);

    void call(QString cmd);

signals:
    void call_signal(QString cmd);
    void signalStop();
    void signalDatasetStatusUpdated(int);
    void signalTryVisualize(QString topic,QString type);

public slots:
    void call_slot(QString cmd);
    void slotShowMessage(QString str,int msgType=0);
    bool slotOpen();
    bool slotOpen(QString file);
    bool slotStart();
    bool slotPause();
    bool slotStop();
    bool slotOneStep();
    bool slotAddSLAM(QString pluginPath);
    bool slotStartDataset(QString dataset);
    void slotUpdate();
    void slotSetSceneRadius(qreal radius);
    void slotSetSceneCenter(qreal x,qreal y,qreal z);
    void slotSetViewPoint(qreal x,qreal y,qreal z,
                            qreal rw,qreal rx,qreal ry,qreal rz);
    void slotDatasetStatusUpdated(int status);
    void slotTryVisualize(QString topic,QString type);

protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void closeEvent(QCloseEvent *event);

    void runSLAMMain();
public:
    void datasetStatusUpdated(const int& status)
    {
        emit signalDatasetStatusUpdated(status);
    }

    void tryVisualize(const GSLAM::Publisher& pub)
    {
        emit signalTryVisualize(pub.getTopic().c_str(),pub.getTypeName().c_str());
    }
protected:

    QMenu    *fileMenu,*exportMenu,*historyMenu,*runMenu;
    QToolBar *toolBar;
    QAction  *openAction,*startAction,*pauseAction,*stopAction,*oneStepAction;

    Dataset                 dataset; // current dataset, load implementations
    FrameVisualizer         *frameVis;
//    SvarWithType<SLAMVisualizerPtr> slamVis;

    QDockWidget             *operateDock;
    QSplitter               *splitterLeft;
    GImageVisualizer        *gimageVis;
//    QTabWidget              *slamTab;
    Win3D                   *win3d;

    int                     status;

    std::string                  historyFile,lastOpenFile,defaultDataset;
    VecParament<std::string> defaultSLAMs;
public:
    Publisher               pub_gui;
};

class SCommandAction : public QAction
{
    Q_OBJECT
public:
    SCommandAction(const QString& cmd,const QString& text="",QMenu* parent=NULL);

public slots:
    void triggerdSlot();

private:
    QString _cmd;
};

class ShowLayerWidget:public QTreeWidget
{
    Q_OBJECT
public:
    ShowLayerWidget(QWidget* parent=NULL);
    void addItem(std::string itemName,int status);
signals:
    void signalAddItem(QString,int status);
    void signalStatusChanged(QString,int);
protected slots:
    void changedSlot(QTreeWidgetItem *item, int column);
    void slotAddItem(QString itemName,int status);
    bool changeLanguage();
};

}

#endif // MAINWINDOW_H
#endif // HAS_QT
