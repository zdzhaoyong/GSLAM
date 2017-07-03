#ifdef HAS_QT
#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QAction>
#include <memory>

namespace GSLAM{

class MainWindowData;
class MainWindow: public QMainWindow
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
    bool slotAddSLAM(QString pluginPath);
    bool slotStartDataset(QString dataset);

protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void closeEvent(QCloseEvent *event);

    void runSLAMMain();

    std::shared_ptr<MainWindowData> _d;
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

}

#endif // MAINWINDOW_H
#endif // HAS_QT
