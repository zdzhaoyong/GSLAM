#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <gui/gl/Win3D.h>

class MainWindow:public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow(){}

    virtual int setupLayout(void);

    pi::gl::Win3D* getWin3D(){return win3d;}

    void call(QString cmd);

signals:
    void call_signal(QString cmd);

protected slots:
    void call_slot(QString cmd);

protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void closeEvent(QCloseEvent *event);

    pi::gl::Win3D* win3d;
};

#endif // MAINWINDOW_H
