#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <memory>

#include <QMenu>
#include <QMainWindow>
#include <QAction>
#include <QTreeWidget>
#include <GSLAM/core/GSLAM.h>

#include "QGLViewer/qglviewer.h"

namespace GSLAM{

class Win3D : public QGLViewer
{
public:
    Win3D(QWidget* parent)
        : QGLViewer(parent){
        _pub_draw=messenger.advertise<Svar>("qviz/draw");
        _sub_update=messenger.subscribe("qviz/update",[this](bool update){
            this->update();
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
    Subscriber  _sub_update;
    Svar        _status;
};


class MessengerAction : public QAction
{
    Q_OBJECT
public:
    MessengerAction(const QString& topic,QMenu* parent=NULL)
        :QAction(parent)
    {
        setText(topic);
        if(parent)
            parent->addAction(this);
        connect(this, SIGNAL(triggered()), this, SLOT(triggerdSlot()));
        _pub=messenger.advertise<bool>(topic.toStdString());
    }

    MessengerAction(const QString& name,QMenu* parent,SvarFunction func)
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
        messenger.publish("gslam.stop",true);
    }

    void datasetStatusUpdated(const int& status)
    {
        emit signalDatasetStatusUpdated(status);
    }

    void addPanel(QWidget* widget);
signals:
    void signalDatasetStatusUpdated(int);

public slots:
    void slotDatasetStatusUpdated(int status);
    void slotOpen(QString filePath);
protected:
    void preparePanels();
    void showPanel(std::string panelName);
    Svar  data;
    QAction *startAction,*pauseAction,*stopAction,*oneStepAction;
};

}

#endif // MAINWINDOW_H
