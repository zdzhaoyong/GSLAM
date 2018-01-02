#ifdef HAS_QT
#include <QKeyEvent>
#include <QMessageBox>
#include <QSplitter>
#include <QMenuBar>
#include <QToolBar>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>

#include "MainWindow.h"
#include "FrameVisualizer.h"

#if defined(HAS_QT)&&defined(HAS_QGLVIEWER)
#include "SLAMVisualizer.h"
#else
class SLAMVisualizer;
#endif

#include "../../core/Svar.h"
#include "../../core/GSLAM.h"
#include "../../core/Dataset.h"
#include "../../core/Svar.h"
#include "../../core/Timer.h"
#include "../../core/VecParament.h"

using namespace std;

namespace GSLAM{

enum ThreadStatus
{
    RUNNING,PAUSE,STOP
};

struct MainWindowData
{
    MainWindowData()
        : frameVis(NULL),status(STOP),historyFile("history.txt"),
          defaultSLAMs(svar.get_var<VecParament<std::string> >("SLAM",VecParament<std::string>())),
          defaultDataset(svar.GetString("Dataset",""))
    {}

    QMenu    *fileMenu,*exportMenu,*historyMenu,*runMenu;
    QToolBar *toolBar;
    QAction  *openAction,*startAction,*pauseAction,*stopAction;

    Dataset                 dataset; // current dataset, load implementations
    FrameVisualizer         *frameVis;
    vector<SLAMVisualizer*> slamVis;

    QDockWidget             *operateDock;
    QSplitter               *splitterLeft;
    QTabWidget              *slamTab;

    std::thread             threadPlay;
    int                     status;

    string                  historyFile,lastOpenFile,defaultDataset;
    VecParament<std::string> defaultSLAMs;
};

void GuiHandle(void *ptr,string cmd,string para)
{
    if(cmd=="MainWindow.Show")
    {
        MainWindow* mainwindow=(MainWindow*)ptr;
        mainwindow->call("Show");
        return;
    }
    else if(cmd=="MainWindow.Update")
    {
        MainWindow* mainwindow=(MainWindow*)ptr;
        mainwindow->call("Update");
        return;
    }
    else if(cmd=="MainWindow.SetRadius")
    {
        MainWindow* mainwindow=(MainWindow*)ptr;
        stringstream sst(para);
        float radius=-1;
        sst>>radius;
        return;
    }

}

////////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(QWidget *parent)
    :QMainWindow(parent),_d(new MainWindowData())
{
    // set window minimum size
    this->setMinimumSize(1366, 700);

    // window title
    setWindowTitle("GSLAM");

    scommand.RegisterCommand("MainWindow.Show",GuiHandle,this);
    scommand.RegisterCommand("MainWindow.Update",GuiHandle,this);
    scommand.RegisterCommand("MainWindow.SetRadius",GuiHandle,this);

    // setup layout
    setupLayout();
    connect(this, SIGNAL(call_signal(QString) ), this, SLOT(call_slot(QString)) );

    if(_d->defaultDataset.size()) slotStartDataset(_d->defaultDataset.c_str());
}

int MainWindow::setupLayout(void)
{
    _d->toolBar=addToolBar(tr("&ToolBar"));
    _d->fileMenu    =menuBar()->addMenu(tr("&File"));
    _d->runMenu     =menuBar()->addMenu(tr("&Run"));

    _d->exportMenu  =new QMenu(tr("&Export"),_d->fileMenu);
    _d->historyMenu =new QMenu(tr("&History"),_d->fileMenu);
    _d->openAction  =new QAction(tr("&Open"),_d->fileMenu);
    _d->startAction =new QAction(tr("&Start"),_d->runMenu);
    _d->pauseAction =new QAction(tr("&Pause"),_d->runMenu);
    _d->stopAction  =new QAction(tr("&Stop"),_d->runMenu);

    _d->fileMenu->addAction(_d->openAction);
    _d->fileMenu->addMenu(_d->exportMenu);
    _d->fileMenu->addMenu(_d->historyMenu);
    _d->runMenu->addAction(_d->startAction);
    _d->runMenu->addAction(_d->pauseAction);
    _d->runMenu->addAction(_d->stopAction);

    _d->toolBar->setMovable(true);
    _d->toolBar->addAction(_d->openAction);
    _d->toolBar->addSeparator();
    _d->toolBar->addAction(_d->startAction);
    _d->toolBar->addAction(_d->pauseAction);
    _d->toolBar->addAction(_d->stopAction);
    _d->pauseAction->setDisabled(true);
    _d->stopAction->setDisabled(true);
    _d->startAction->setDisabled(true);

    QString iconFolder=":icon";
    _d->openAction->setIcon(QIcon(iconFolder+"/open.png"));
    _d->exportMenu->setIcon(QIcon(iconFolder+"/export.png"));
    _d->startAction->setIcon(QIcon(iconFolder+"/start.png"));
    _d->pauseAction->setIcon(QIcon(iconFolder+"/pause.png"));
    _d->stopAction->setIcon(QIcon(iconFolder+"/stop.png"));

    if(_d->historyFile.size())
    {
        ifstream ifs(_d->historyFile);
        if(ifs.is_open())
        {
            string line;
            while(getline(ifs,line))
            {
                if(line.empty()) continue;
                _d->historyMenu->addAction(new SCommandAction(QString::fromStdString("MainWindow Open "+line),
                                                              QString::fromStdString(line),_d->historyMenu));
            }
        }
    }

    _d->operateDock   =new QDockWidget(this);
    _d->slamTab       =new QTabWidget(this);
    _d->splitterLeft    =new QSplitter(Qt::Vertical,_d->operateDock);

    _d->operateDock->setWindowTitle("SideBar");
    _d->operateDock->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);
    _d->operateDock->setWidget(_d->splitterLeft);
    addDockWidget(Qt::LeftDockWidgetArea,_d->operateDock);
    _d->frameVis      =new FrameVisualizer(_d->splitterLeft);

    for(std::string plugin:_d->defaultSLAMs.data)
    {
        slotAddSLAM(plugin.c_str());
    }

    setCentralWidget(_d->slamTab);

    connect(_d->openAction,SIGNAL(triggered(bool)),this,SLOT(slotOpen()));
    connect(_d->startAction,SIGNAL(triggered(bool)),this,SLOT(slotStart()));
    connect(_d->pauseAction,SIGNAL(triggered(bool)),this,SLOT(slotPause()));
    connect(_d->stopAction,SIGNAL(triggered(bool)),this,SLOT(slotStop()));
    connect(this,SIGNAL(signalStop()),this,SLOT(slotStop()));
    return 0;
}

void MainWindow::keyPressEvent(QKeyEvent *e)
{
    switch(e->key())
    {
    case Qt::Key_Space:
        break;

    case Qt::Key_Escape:
        close();
        break;

    default:
        svar.i["KeyPressMsg"] = e->key();
        break;
    }
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
#if 0
    // 1 - left
    // 2 - right
    // 4 - middle
    printf("window pressed, %d, %d, %d\n", event->button(), event->pos().x(), event->pos().y());

    if( event->button() == 1 ) {

    }
#endif
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if(!slotStop()) return;
//    this->~MainWindow();
}

void MainWindow::call(QString cmd)
{
    emit call_signal(cmd);
}

void MainWindow::call_slot(QString cmd)
{
    if("Show"==cmd)   show();
    if("Update"==cmd)
    {
    }
    else
        scommand.Call(cmd.toStdString());
}

void MainWindow::slotShowMessage(QString str,int msgType)
{
    QMessageBox message(QMessageBox::NoIcon, "GSLAM",str);
    message.setIconPixmap(QIcon(":icon/rtmapper.png").pixmap(QSize(64,64)));
    message.exec();
}

bool MainWindow::slotOpen()
{
    return slotOpen("");
}

bool MainWindow::slotOpen(QString filePath)
{
    if(!filePath.size())
    {
        filePath= QFileDialog::getOpenFileName(this, tr("Choose a file.\n"),
                                                       _d->lastOpenFile.c_str(),
                                                      "Allfile(*.*);;");
    }
    QFileInfo info(filePath);
    if(!info.exists()) return false;
    if(info.suffix()=="so"||info.suffix()=="dll"){
        return slotAddSLAM(filePath);
    }
    else return slotStartDataset(filePath);
    return true;
}

bool MainWindow::slotStart()
{
    if(_d->status==PAUSE)
    {
        _d->status=RUNNING;
    }
    else if(_d->status==RUNNING)
    {
        // still running and need to stop first
        if(!slotStop())
        {
            return false;
        }
    }

    if(_d->status==STOP){
        if(!_d->dataset.isOpened())
        {
            slotShowMessage(tr("Please open a dataset first!\n"));
        }
        _d->status=RUNNING;
        _d->threadPlay=std::thread(&MainWindow::runSLAMMain,this);
    }
    _d->startAction->setDisabled(true);
    _d->pauseAction->setDisabled(false);
    _d->stopAction->setDisabled(false);
    return true;
}

bool MainWindow::slotPause()
{
    if(_d->status!=RUNNING) return false;
    _d->status=PAUSE;
    _d->startAction->setDisabled(false);
    _d->pauseAction->setDisabled(true);
    _d->stopAction->setDisabled(false);
    return true;
}

bool MainWindow::slotStop()
{
    if(_d->status==STOP) return false;
    _d->status=STOP;
    while(!_d->threadPlay.joinable()) GSLAM::Rate::sleep(0.01);
    _d->threadPlay.join();
    _d->startAction->setDisabled(false);
    _d->pauseAction->setDisabled(true);
    _d->stopAction->setDisabled(true);

#if defined(HAS_QT)&&defined(HAS_QGLVIEWER)
        for(SLAMVisualizer* vis:_d->slamVis)
        {
            vis->releaseSLAM();
        }
#endif
    return true;
}

bool MainWindow::slotAddSLAM(QString pluginPath)
{
#if defined(HAS_QT)&&defined(HAS_QGLVIEWER)
    SLAMVisualizer* slamVis=new SLAMVisualizer(this,pluginPath);
    if(slamVis->slam()&&slamVis->slam()->valid())
    {
        _d->slamVis.push_back(slamVis);
        _d->slamTab->addTab(slamVis,slamVis->slam()->type().c_str());
        return true;
    }
    delete slamVis;
#endif
    return false;
}

bool MainWindow::slotStartDataset(QString dataset)
{
    if(!_d->dataset.open(dataset.toStdString()))
    {
        slotShowMessage(tr("Failed to open dataset ")+dataset);
        return false;
    }
    _d->frameVis->setFrame(_d->dataset.grabFrame());
    _d->startAction->setEnabled(true);
    return true;
}

void MainWindow::runSLAMMain()
{
    Rate rate(svar.GetInt("Frequency",100));
    while(_d->status!=STOP)
    {
        rate.sleep();
        if(_d->status==PAUSE)
        {
            continue;
        }

        FramePtr frame=_d->frameVis->curFrame();
        if(!frame) break;
#if defined(HAS_QT)&&defined(HAS_QGLVIEWER)
        for(SLAMVisualizer* vis:_d->slamVis)
        {
            string str=vis->slam()->type()+"::Track";
            timer.enter(str.c_str());
            vis->slam()->track(frame);
            timer.leave(str.c_str());
        }
#endif
        _d->frameVis->setFrame(_d->dataset.grabFrame());
    }
    std::cerr<<"Play thread stoped."<<endl;
    emit signalStop();
}

SCommandAction::SCommandAction(const QString &cmd, const QString &text, QMenu *parent)
    :QAction(text,(QObject*)parent),_cmd(cmd)
{
      setObjectName(text);
      if(parent)
          parent->addAction(this);
        connect(this, SIGNAL(triggered()), this, SLOT(triggerdSlot()));
//    connect(((QAction*)this), &QAction::triggered, this, SCommandAction::triggerdSlot);
}

void SCommandAction::triggerdSlot()
{
//    std::cerr<<"SCommandAction::triggerdSlot";
    scommand.Call(_cmd.toStdString());
}
}

#endif
