#include "../core/Svar.h"
#include "../core/Timer.h"
#include "Tests/gtest.h"
#if defined(HAS_QT)
#include "GUI/MainWindow.h"
#include <QApplication>
#endif

using namespace std;

int main(int argc,char** argv)
{
    timer.enter("Main");
    svar.ParseMain(argc,argv);
    int    ret=0;

#if defined(HAS_QT)
    string act=svar.GetString("Act","SLAM");
    if("SLAM"==act)
    {
        {
            QApplication app(svar.i["argc"],(char**)svar.GetPointer("argv"));
            GSLAM::MainWindow mainwindow;
            mainwindow.show();
//            for(int i=1;i<argc;i++)
//            {
//                string arg=argv[i];
//                if(arg.find('=')==string::npos&&arg.front()!='-')
//                {
//                    mainwindow.slotOpen(arg.c_str());
//                }

//            }
            ret=app.exec();
        }
    }
#else
    string act=svar.GetString("Act","Tests");
#endif
    if("Tests"==act)
    {
        testing::InitGoogleTest(&svar.i["argc"],(char**)svar.GetPointer("argv"));
        ret=RUN_ALL_TESTS();
    }
    timer.leave("Main");
    return ret;
}

