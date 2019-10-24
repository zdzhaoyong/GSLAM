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

namespace GSLAM{


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
    void addMenu(Svar menu,QMenu* parent=nullptr);
    void addTool(Svar tool);
    void uiRun(Svar run);
    void shutdown(){
        emit signalClose();
    }
signals:
    void signalDatasetStatusUpdated(int);
    void signalUiRun(Svar run);
    void signalClose();

public slots:
    void slotDatasetStatusUpdated(int status);
    void slotOpen(QString filePath);
    void slotUiRun(Svar run){
        run();
    }
protected:
    void preparePanels();
    void showPanel(std::string panelName);
    Svar  data;
    QTabWidget    *_tab;
    QToolBar      *_toolBar;
    QAction       *startAction,*pauseAction,*stopAction,*oneStepAction;
};

}

#endif // MAINWINDOW_H
