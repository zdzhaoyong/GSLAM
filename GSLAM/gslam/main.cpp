#include "../core/Svar.h"
#include "../core/Timer.h"
#include "Tests/gtest.h"
#include <GSLAM/core/MemoryMetric.inc>
#if defined(HAS_QT)
#include "GUI/MainWindow.h"
#include <QApplication>
#endif

using namespace std;

int main(int argc,char** argv)
{
    svar.Arg<std::string>("Act","Tests","The default action going to excute. use \"Act=Tests\" to excute module testing.");
    svar.Arg<std::string>("SLAM","","The SLAM plugin path,eg. libgslam.so");
    svar.Arg<std::string>("Dataset","","The Dataset location with extesion.");
    svar.Arg<double>("PlaySpeed",1.,"The Dataset play speed factor, 1 means the original speed.");
    timer.enter("Main");
    auto unParsed=svar.ParseMain(argc,argv);
    int    ret=0;

    string& act=svar.GetString("Act","Tests");
#if defined(HAS_QT)
    if(unParsed.size()
            ||!svar.GetString("SLAM","").empty()
            ||!svar.GetString("Dataset").empty()) act="SLAM";
    if("SLAM"==act)
    {
        {
            QApplication app(svar.GetInt("argc"),(char**)svar.GetPointer("argv"));
            GSLAM::MainWindow mainwindow;
            mainwindow.show();
            for(std::string arg:unParsed)
            {
                mainwindow.slotOpen(arg.c_str());
            }
            ret=app.exec();
        }
    }
#endif

    if("Tests"==act)
    {
        testing::InitGoogleTest(&svar.GetInt("argc"),(char**)svar.GetPointer("argv"));
        ret=RUN_ALL_TESTS();
    }
    timer.leave("Main");
    return ret;
}

