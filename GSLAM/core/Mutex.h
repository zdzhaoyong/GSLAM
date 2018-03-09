#ifndef GSLAM_MUTEX__H
#define GSLAM_MUTEX__H

#ifdef HAS_PIL0

#include <pil/base/Thread/Thread.h>
#include <pil/base/Thread/Event.h>

namespace GSLAM{

typedef pi::MutexRW     MutexRW;
typedef pi::ReadMutex   ReadMutex;
typedef pi::WriteMutex  WriteMutex;
typedef pi::Event       Event;

}

#else
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace GSLAM{
typedef std::mutex Mutex;
typedef std::mutex MutexRW;
typedef std::unique_lock<MutexRW> ReadMutex;
typedef std::unique_lock<MutexRW> WriteMutex;

class Event
{
public:
    Event(bool autoReset = true):_auto(autoReset),_state(false){

    }
    ~Event(){}

    void set(){notify_all();}

    void wait(){
        if(_auto)
            _state = false;
        if (!_state)
        {
            std::unique_lock<std::mutex> _lock(_mutex);
            _cond.wait(_lock);
        }
    }

    void notify_all()
    {
        _cond.notify_all();
        _state = true;
    }

    void notify_once()
    {
        _cond.notify_one();
        _state = true;
    }

    void reset(){
        std::unique_lock<std::mutex> _lock(_mutex);
        _state=false;
    }

private:
    Event(const Event&);
    Event& operator = (const Event&);

    bool            _auto;
    std::atomic<bool> _state; 
    std::mutex      _mutex;
    std::condition_variable _cond;
};

}
#endif

#endif
