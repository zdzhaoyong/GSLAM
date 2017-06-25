#ifndef SYSTEM_H
#define SYSTEM_H
#include "../core/GSLAM.h"
#include "../core/SharedLibrary.h"

class System
{
public:
    System();
    void run();
    void SLAMMain();
private:
    SPtr<GSLAM::SharedLibrary> _slamPlugin;
    GSLAM::SLAMPtr             _slam;
};

#endif // SYSTEM_H
