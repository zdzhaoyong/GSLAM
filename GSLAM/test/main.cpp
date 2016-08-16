#include <base/Svar/Svar.h>
#include <QApplication>
#include "System.h"

int main(int argc,char** argv)
{
    svar.ParseMain(argc,argv);
    if(svar.GetInt("WithQt"))
    {
        QApplication app(argc,argv);
        System system;
        system.start();
        return app.exec();
    }
    else
    {
        System system;
        system.run();
    }

    return 0;
}

