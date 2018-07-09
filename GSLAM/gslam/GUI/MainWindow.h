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

#include "../../core/Svar.h"
#include "../../core/GSLAM.h"
#include "../../core/Dataset.h"
#include "../../core/Svar.h"
#include "../../core/Timer.h"
#include "../../core/VecParament.h"

#include "QGLViewer/qglviewer.h"

namespace GSLAM{

class Win3D : public QGLViewer
{
public:
    Win3D(QWidget* parent,std::vector<SLAMVisualizerPtr>* visualizers)
        : QGLViewer(parent),_visualizers(visualizers),_fastDraw(svar.GetInt("FastDrawing",0)){}

    virtual void draw()
    {
        _fastDraw=0;
        for(SLAMVisualizerPtr& vis : *_visualizers)
            vis->draw();
    }

    virtual void fastDraw()
    {
        _fastDraw=1;
        for(SLAMVisualizerPtr& vis : *_visualizers)
            vis->draw();
    }

    int&                            _fastDraw;
    std::vector<SLAMVisualizerPtr>* _visualizers;
};

class MainWindow: public QMainWindow,GObjectHandle
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow(){slotStop();}

    virtual int setupLayout(void);

    void call(QString cmd);

signals:
    void call_signal(QString cmd);
    void signalStop();

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

protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void closeEvent(QCloseEvent *event);

    void runSLAMMain();

    QMenu    *fileMenu,*exportMenu,*historyMenu,*runMenu;
    QToolBar *toolBar;
    QAction  *openAction,*startAction,*pauseAction,*stopAction,*oneStepAction;

    Dataset                 dataset; // current dataset, load implementations
    FrameVisualizer         *frameVis;
    std::vector<SLAMVisualizerPtr> slamVis;

    QDockWidget             *operateDock;
    QSplitter               *splitterLeft;
//    QTabWidget              *slamTab;
    Win3D                   *win3d;

    std::thread             threadPlay;
    int                     status;

    std::string                  historyFile,lastOpenFile,defaultDataset;
    VecParament<std::string> defaultSLAMs;
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
    void addItem(QString itemName,int status);
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
