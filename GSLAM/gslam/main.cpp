#include "../core/Svar.h"
#include "Tests/gtest.h"
#if defined(HAS_QT)
#include "GUI/MainWindow.h"
#include <QApplication>
#endif

using namespace std;

int main(int argc,char** argv)
{
    svar.ParseMain(argc,argv);

#if defined(HAS_QT)
    string act=svar.GetString("Act","SLAM");
    if("SLAM"==act)
    {
        QApplication app(svar.i["argc"],(char**)svar.GetPointer("argv"));
        GSLAM::MainWindow* mainwindow=new GSLAM::MainWindow(NULL);
        mainwindow->show();
        return app.exec();
    }
#else
    string act=svar.GetString("Act","Tests");
#endif
    if("Tests"==act)
    {
        testing::InitGoogleTest(&svar.i["argc"],(char**)svar.GetPointer("argv"));
        return RUN_ALL_TESTS();
    }

    cerr<<"Action "<<act<<" is not supported.\n";
    return 0;
}

