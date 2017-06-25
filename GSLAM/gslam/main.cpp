#include "../core/Svar.h"
#include "System.h"

#ifdef HAS_QT
#include <QApplication>
#endif

int main(int argc,char** argv)
{
    svar.ParseMain(argc,argv);

#ifdef HAS_QT
    if(svar.GetInt("WithQt",1))
    {
        QApplication app(argc,argv);
        System system;
        system.run();
        return app.exec();
    }
    else
#endif
    {
        System system;
        system.run();
    }

    return 0;
}

