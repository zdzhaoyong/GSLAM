#include "System.h"
#include <set>

#include "../core/Svar.h"
#include "Tests/gtest.h"
#if defined(HAS_QT)
#include "GUI/MainWindow.h"
#endif

using namespace std;

int excuteTest()
{
    testing::InitGoogleTest(&svar.i["argc"],(char**)svar.GetPointer("argv"));
    return RUN_ALL_TESTS();
}

System::System()
{
}

void System::run()
{
    string act=svar.GetString("Act","SLAM");
#if defined(HAS_QT)
    if("SLAM"==act)
    {
        SLAMMain();
    }
#endif
    if("Tests"==act)
    {
        excuteTest();
    }
}

void System::SLAMMain()
{
    MainWindow* mainwindow=new MainWindow(NULL);
    mainwindow->show();
}

