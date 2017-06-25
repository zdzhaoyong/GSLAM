#include "MainWindow.h"
#include "../../core/Svar.h"
#include <QKeyEvent>

using namespace std;

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
MainWindow::MainWindow(QWidget *parent):QMainWindow(parent)
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
}

int MainWindow::setupLayout(void)
{
    QTabWidget* m_tabWidget = new QTabWidget(this);

    /// 3. Overall layout
    setCentralWidget(m_tabWidget);
    return 0;
}

void MainWindow::keyPressEvent(QKeyEvent *e)
{
    switch(e->key())
    {
    case Qt::Key_Space:
        break;

    case Qt::Key_Escape:
        exit(0);
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
    svar.ParseLine("System.Stop");
    exit(0);
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
