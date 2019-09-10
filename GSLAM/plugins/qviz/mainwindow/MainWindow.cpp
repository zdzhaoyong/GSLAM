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

    _toolBar=addToolBar(tr("&ToolBar"));
    _tab=new QTabWidget(this);

    auto fileMenu    =menuBar()->addMenu(tr("&File"));
    auto runMenu     =menuBar()->addMenu(tr("&Run"));

    auto exportMenu  =new QMenu(tr("&Export"),fileMenu);
    auto historyMenu =new QMenu(tr("&History"),fileMenu);
    auto openAction  =new MessengerAction(tr("&Open"),fileMenu,Svar::lambda([this](){
        this->slotOpen("");
    }));
    startAction  =new MessengerAction(tr("&Start"),"qviz/start",runMenu);
    pauseAction  =new MessengerAction(tr("&Pause"),"qviz/pause",runMenu);
    stopAction   =new MessengerAction(tr("S&top"),"qviz/stop",runMenu);
    oneStepAction=new MessengerAction(tr("&Step"),"qviz/step",runMenu);

    fileMenu->addAction(openAction);
    fileMenu->addMenu(exportMenu);
    fileMenu->addMenu(historyMenu);
    runMenu->addAction(startAction);
    runMenu->addAction(pauseAction);
    runMenu->addAction(stopAction);
    runMenu->addAction(oneStepAction);

    _toolBar->setMovable(true);
    _toolBar->addAction(openAction);
    _toolBar->addSeparator();
    _toolBar->addAction(startAction);
    _toolBar->addAction(pauseAction);
    _toolBar->addAction(oneStepAction);
    _toolBar->addAction(stopAction);
    _toolBar->addSeparator();
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

    setCentralWidget(_tab);
    auto win3d=new Win3D(this);
    win3d->setObjectName("3D");
    addTab(win3d);

    connect(this,SIGNAL(signalDatasetStatusUpdated(int)),
            this,SLOT(slotDatasetStatusUpdated(int)));
    connect(this,SIGNAL(signalUiRun(Svar*)),
            this,SLOT(slotUiRun(Svar*)));

    data["config"]=config;
    preparePanels();
    showPanel("displays");
}


void MainWindow::addPanel(QWidget* widget)
{
    QDockWidget* dock=dynamic_cast<QDockWidget*>(widget);
    if(!dock){
        dock=new QDockWidget(widget->objectName(),this);
        dock->setWidget(widget);
    }
    addDockWidget(Qt::LeftDockWidgetArea,dock);
}

void MainWindow::addTab(QWidget* widget)
{
    _tab->addTab(widget,widget->objectName());
}

void MainWindow::addMenu(Svar menuVar,QMenu* parent)
{
    if(!menuVar.exist("name")) return ;
    std::string name=menuVar.get<std::string>("name","NoName");
    std::string icon=menuVar.get<std::string>("icon","");
    std::vector<Svar> children=menuVar.get<std::vector<Svar>>("children",{});
    std::vector<Svar> actions =menuVar.get<std::vector<Svar>>("actions",{});
    QMenu* menu=nullptr;
    if(!parent)
        menu=menuBar()->addMenu(name.c_str());
    else{
        menu=new QMenu(name.c_str(),parent);
        parent->addMenu(menu);
    }

    if(icon.size()) menu->setIcon(QIcon(icon.c_str()));

    for(Svar child:children) addMenu(child,menu);

    for(Svar tool:actions){
        std::string name =tool.get<std::string>("name","NoName");
        std::string icon =tool.get<std::string>("icon","");
        std::string topic=tool.get<std::string>("topic","");
        Svar        func =tool["callback"];
        QAction* action=nullptr;
        if(func.isFunction())
            action=new MessengerAction(name.c_str(),nullptr,func);
        else if(!topic.empty())
            action=new MessengerAction(name.c_str(),topic,nullptr);
        else return;

        if(!icon.empty())
            action->setIcon(QIcon(icon.c_str()));
        menu->addAction(action);
    }
}

void MainWindow::uiRun(Svar func)
{
    emit signalUiRun(new Svar(func));
}

void MainWindow::addTool(Svar tool)
{
    std::string name =tool.get<std::string>("name","NoName");
    std::string icon =tool.get<std::string>("icon","");
    std::string topic=tool.get<std::string>("topic","");
    Svar        func =tool["callback"];
    QAction* action=nullptr;
    if(func.isFunction())
        action=new MessengerAction(name.c_str(),nullptr,func);
    else if(!topic.empty())
        action=new MessengerAction(name.c_str(),topic,nullptr);
    else return;

    if(!icon.empty())
        action->setIcon(QIcon(icon.c_str()));

    _toolBar->addAction(action);
}

void MainWindow::preparePanels()
{
    Svar config=data["config"];
    regex is_rviz_plugin("^(?:|lib)?qviz_([a-zA-Z\\d_]+).(?:|so|dll|dylib)$");
    auto folder=absolute(path(config.get<char**>("argv",nullptr)[0]).parent_path());
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
    if(!config["gslam"]["panels"].isObject()) return;
    std::map<std::string,Svar> panels=config["gslam"]["panels"].castAs<std::map<std::string,Svar>>();
    QMenu* menuPanels=menuBar()->addMenu(tr("&Panels"));
    for(std::pair<std::string,Svar> p:panels){
        QAction* action=new MessengerAction(p.first.c_str(),menuPanels,Svar::lambda([this,p](){
            showPanel(p.first);
        }));
        menuPanels->addAction(action);
    }
}

void MainWindow::showPanel(std::string panelName)
{
    if(data["panels"][panelName].is<QWidget*>())
    {
        if(data["panels"][panelName].as<QWidget*>()->isVisible())
            return;
        else
            data["panels"][panelName].as<QWidget*>()->setVisible(true);
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
    if(filePath.isEmpty()) return;

    messenger.publish<std::string>("qviz/open",filePath.toStdString());
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
