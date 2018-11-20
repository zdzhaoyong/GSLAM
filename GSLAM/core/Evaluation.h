#ifndef GSLAM_CORE_EVALUATION
#define GSLAM_CORE_EVALUATION

#include <unordered_map>

#include <GSLAM/core/Messenger.h>
#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/Timer.h>
#include <GSLAM/core/MemoryMetric.h>
#include "CPUMetric.h"

namespace GSLAM {

class Evaluation
{
public:
    struct FrameData{
        double timestamp;
        double time_used;
        uint   mem_usage,mem_allocate;
        float  cpu_usage;
        SIM3   pose;
    };

    Evaluation(){
        auto& msg=Messenger::instance();
        _sub_curframe=msg.subscribe("curframe",0,&Evaluation::startFrame,this);
        _sub_processed_frame=msg.subscribe("processed_frame",0,&Evaluation::endFrame,this);
        _pub_framedata=msg.advertise<FrameData>("evaluation",2);
        _start_mem=MemoryMetric::instanceCPU().usage();
        _start_alloc=MemoryMetric::instanceCPU().count();
        MemoryMetric::instanceCPU().enable();
    }

    ~Evaluation(){
        std::ofstream ofs(svar.GetString("evaluation","evaluation.txt"));
        std::ofstream traj(svar.GetString("trajectory","eva_traj.txt"));
//        std::map<FrameID,FrameData> vec;
//        vec.reserve(_data.size());
//        for(auto it:_data) vec.push_back(it);
//        std::sort(vec.begin(),vec.end());
        ofs.precision(18);
        traj.precision(18);
        for(auto it:_data)
        {
            ofs<<it.first<<","
              <<it.second.timestamp<<","
             <<it.second.time_used<<","
            <<it.second.mem_usage<<","
            <<it.second.mem_allocate<<","
            <<it.second.cpu_usage<<","
            <<it.second.pose.get_se3()<<std::endl;
            traj<<it.second.timestamp<<" "<<it.second.pose.get_se3()<<std::endl;
        }
    }

    void startFrame(const MapFrame& fr){
        FrameData& d=_data[fr.id()];
        d.timestamp=TicToc::timestamp();
    }

    void endFrame(const MapFrame& fr)
    {
        FrameData& d=_data[fr.id()];
        d.time_used=TicToc::timestamp()-d.timestamp;
        d.timestamp=fr.timestamp();
        d.mem_allocate=MemoryMetric::instanceCPU().count();
        d.mem_usage=MemoryMetric::instanceCPU().usage();
        d.cpu_usage=_cpumetric.usage();
        d.pose=fr.getPoseScale();

        _pub_framedata.publish(d);
    }

    CPUMetric                             _cpumetric;
    std::map<FrameID,FrameData> _data;
    Subscriber                  _sub_curframe,_sub_processed_frame;
    Publisher                   _pub_framedata;
    size_t                      _start_mem,_start_alloc;
};

}
#endif
