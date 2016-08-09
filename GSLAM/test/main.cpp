#include <base/Svar/Svar.h>
#include "System.h"

int main(int argc,char** argv)
{
    svar.ParseMain(argc,argv);
    if(svar.GetInt("WithQt"))
    {

    }
    else
    {
        System system;
        system.run();
    }

    return 0;
}

