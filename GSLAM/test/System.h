#ifndef SYSTEM_H
#define SYSTEM_H
#include <base/Thread/Thread.h>

class System : public pi::Thread
{
public:
    System();
    virtual void run();
};

#endif // SYSTEM_H
