#include <QKeyEvent>
#include <QMessageBox>
#include <QSplitter>
#include <QMenuBar>
#include <QToolBar>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>

#include <regex>

#include "MainWindow.h"
#include "filesystem.hpp"

using namespace std;
using namespace ghc::filesystem;

namespace GSLAM{

////////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(QWidget *parent,Svar config)
    : QMainWindow(parent)
{
    // set window minimum size
    this->setMinimumSize(1366, 700);

    // window title
    setWindowTitle("GSLAM");

    auto toolBar=addToolBar(tr("&ToolBar"));

    auto fileMenu    =menuBar()->addMenu(tr("&File"));
    auto runMenu     =menuBar()->addMenu(tr("&Run"));

    auto exportMenu  =new QMenu(tr("&Export"),fileMenu);
    auto historyMenu =new QMenu(tr("&History"),fileMenu);
    auto openAction  =new MessengerAction(tr("&Open"),fileMenu,[this](){
        this->slotOpen("");
    });
    startAction =new MessengerAction(tr("&Start"),runMenu,[](){
        messenger.publish<std::string>("dataset/control","Start");
    });
    pauseAction =new MessengerAction(tr("&Pause"),runMenu,[](){
        messenger.publish<std::string>("dataset/control","Pause");
    });
    stopAction  =new MessengerAction(tr("S&top"),runMenu,[](){
        messenger.publish<std::string>("dataset/control","Stop");
    });
    oneStepAction=new MessengerAction(tr("&Step"),runMenu,[](){
        messenger.publish<std::string>("dataset/control","Step");
    });

    fileMenu->addAction(openAction);
    fileMenu->addMenu(exportMenu);
    fileMenu->addMenu(historyMenu);
    runMenu->addAction(startAction);
    runMenu->addAction(pauseAction);
    runMenu->addAction(stopAction);
    runMenu->addAction(oneStepAction);

    toolBar->setMovable(true);
    toolBar->addAction(openAction);
    toolBar->addSeparator();
    toolBar->addAction(startAction);
    toolBar->addAction(pauseAction);
    toolBar->addAction(oneStepAction);
    toolBar->addAction(stopAction);
    pauseAction->setDisabled(true);
    stopAction->setDisabled(true);
    startAction->setDisabled(true);
    oneStepAction->setDisabled(true);

    QString iconFolder=":icon";
    openAction->setIcon(QIcon(iconFolder+"/open.png"));
    exportMenu->setIcon(QIcon(iconFolder+"/export.png"));
    startAction->setIcon(QIcon(iconFolder+"/start.png"));
    pauseAction->setIcon(QIcon(iconFolder+"/pause.png"));
    stopAction->setIcon(QIcon(iconFolder+"/stop.png"));
    oneStepAction->setIcon(QIcon(iconFolder+"/startPause.png"));

    setCentralWidget(new Win3D(this));

    connect(this,SIGNAL(signalDatasetStatusUpdated(int)),
            this,SLOT(slotDatasetStatusUpdated(int)));

    data["config"]=config;
    preparePanels();
    showPanel("displays");
}


void MainWindow::addPanel(QWidget* widget)
{
    QDockWidget* dock=dynamic_cast<QDockWidget*>(widget);
    if(!dock){
        dock=new QDockWidget(this);
        dock->setWidget(widget);
    }
    addDockWidget(Qt::LeftDockWidgetArea,dock);
}

void MainWindow::preparePanels()
{
    Svar config=data["config"];
    regex is_rviz_plugin("^(?:|lib)?qviz_([a-zA-Z\\d_]+).(?:|so|dll|dylib)$");
    auto folder=absolute(path(config.get<char**>("argv",nullptr)[0]).root_directory());
    for(auto fileit:directory_iterator(folder))
    {
        smatch result;
        std::string filename = fileit.path().filename();
        if(std::regex_match(filename,result,is_rviz_plugin))
        {
            Svar var=Registry::load(filename);
            Svar qviz=var["gslam"]["panels"];
            if(!qviz.isObject()) continue;
            if(config["gslam"]["panels"].isObject())
                config["gslam"]["panels"].as<SvarObject>().update(qviz);
            else
                config["gslam"]["panels"]=qviz;
        }
    }
    std::map<std::string,Svar> panels=config["gslam"]["panels"].castAs<std::map<std::string,Svar>>();
    QMenu* menuPanels=menuBar()->addMenu(tr("&Panels"));
    for(std::pair<std::string,Svar> p:panels){
        QAction* action=new MessengerAction(p.first.c_str(),menuPanels,[this,p](){
            showPanel(p.first);
        });
        menuPanels->addAction(action);
    }
}

void MainWindow::showPanel(std::string panelName)
{
    if(data["panels"][panelName].is<QWidget*>())
    {
        if(data["panels"][panelName].as<QWidget*>()->isVisible())
            return;
    }
    Svar config=data["config"];
    Svar func=config["gslam"]["panels"][panelName];
    if(!func.isFunction()) return;
    QWidget* widget=func((QWidget*)this,config).castAs<QWidget*>();
    data["panels"][panelName]=widget;
    addPanel(widget);
}

void MainWindow::slotOpen(QString filePath)
{
    if(!filePath.size())
    {
        filePath= QFileDialog::getOpenFileName(this, tr("Choose a file.\n"),
                                                       "",
                                                      "Allfile(*.*);;");
    }
    messenger.publish<std::string>("dataset/control","Open "+filePath.toStdString());
}

void MainWindow::slotDatasetStatusUpdated(int status)
{
    enum Status{
        READY,PLAYING,PAUSING,PAUSED,FINISHING,FINISHED
    };
    switch (status) {
    case READY:{
        startAction->setEnabled(true);
        oneStepAction->setEnabled(true);
    }
        break;
    case PLAYING:{
        startAction->setDisabled(true);
        pauseAction->setDisabled(false);
        stopAction->setDisabled(false);
        oneStepAction->setDisabled(true);
    }
        break;
    case PAUSED:{
        startAction->setDisabled(false);
        pauseAction->setDisabled(true);
        stopAction->setDisabled(false);
        oneStepAction->setDisabled(false);
    }
        break;
    case FINISHING:{

    }
        break;
    case FINISHED:{
        startAction->setDisabled(true);
        pauseAction->setDisabled(true);
        stopAction->setDisabled(true);
        oneStepAction->setDisabled(true);
    }
        break;
    default:
        break;
    }
}

}
