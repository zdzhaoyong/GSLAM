#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <memory>

#include <QMenu>
#include <QMainWindow>
#include <QAction>
#include <QTreeWidget>
#include <QTabWidget>
#include <QCloseEvent>
#include <GSLAM/core/GSLAM.h>

#include "QGLViewer/qglviewer.h"

namespace GSLAM{

class Win3D : public QGLViewer
{
public:
    Win3D(QWidget* parent)
        : QGLViewer(parent){
        _pub_draw=messenger.advertise<Svar>("qviz/gl_draw");
        _sub_update=messenger.subscribe("qviz/gl_update",[this](bool update){
            this->update();
        });
        _sub_radius=messenger.subscribe("qviz/gl_radius",[this](double radius){
            this->setSceneRadius(radius);
        });
        _sub_center=messenger.subscribe("qviz/gl_center",[this](Point3d c){
            this->setSceneCenter(qglviewer::Vec(c.x,c.y,c.z));
        });
        _sub_pose=messenger.subscribe("qviz/gl_camera_pose",[this](SE3 pose){
                GSLAM::Point3d t=pose.get_translation();
                GSLAM::SO3 r=pose.get_rotation();
                this->camera()->setPosition(qglviewer::Vec(t.x,t.y,t.z));
                this->camera()->setOrientation(qglviewer::Quaternion(r.w,r.z,-r.y,-r.x));
        });
    }

    virtual void draw()
    {
        _status["fastDraw"]=false;
        _pub_draw.publish(_status);
    }

    virtual void fastDraw()
    {
        _status["fastDraw"]=true;
        _pub_draw.publish(_status);
    }

    Publisher   _pub_draw;
    Subscriber  _sub_update,_sub_radius,_sub_center,_sub_pose;
    Svar        _status;
};


class MessengerAction : public QAction
{
    Q_OBJECT
public:
    MessengerAction(const QString& name,std::string topic,QMenu* parent=NULL)
        :QAction(parent)
    {
        setText(name);
        if(parent)
            parent->addAction(this);
        connect(this, SIGNAL(triggered()), this, SLOT(triggerdSlot()));
        _pub=messenger.advertise<bool>(topic);
    }

    explicit MessengerAction(const QString& name,QMenu* parent,Svar func)
        :QAction(parent){
        _func=func;
        setText(name);
        if(parent)
            parent->addAction(this);
        connect(this, SIGNAL(triggered()), this, SLOT(triggerdSlot()));
    }
public slots:
    void triggerdSlot(){
        if(_func.isFunction()) _func();
        else _pub.publish(true);
    }
private:
    Publisher _pub;
    Svar      _func;
};

class MainWindow: public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0,Svar config=svar);

    virtual ~MainWindow(){
    }

    void datasetStatusUpdated(const int& status)
    {
        emit signalDatasetStatusUpdated(status);
    }

    void addPanel(QWidget* widget);
    void addTab(QWidget* widget);
    void addMenu(Svar menu);
    void addTool(Svar tool);
    void uiRun(Svar run);
signals:
    void signalDatasetStatusUpdated(int);
    void signalUiRun(Svar* run);

public slots:
    void slotDatasetStatusUpdated(int status);
    void slotOpen(QString filePath);
    void slotUiRun(Svar* run){
        (*run)();
        delete run;
    }
protected:
    void preparePanels();
    void showPanel(std::string panelName);
    void closeEvent(QCloseEvent* e)
    {
        messenger.publish("messenger/stop",true);
        close();
    }
    Svar  data;
    QTabWidget    *_tab;
    QToolBar      *_toolBar;
    QAction       *startAction,*pauseAction,*stopAction,*oneStepAction;
};

}

#endif // MAINWINDOW_H
