#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>

class MainWindow:public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    virtual ~MainWindow(){}

    virtual int setupLayout(void);

    void call(QString cmd);

signals:
    void call_signal(QString cmd);

protected slots:
    void call_slot(QString cmd);

protected:
    void keyPressEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void closeEvent(QCloseEvent *event);

};

#endif // MAINWINDOW_H
