#include <GSLAM/core/GSLAM.h>
#include "CPUMetric.h"

using namespace GSLAM;

int metric_cpu(Svar config){
    std::string slam_name=config.arg<std::string>("slam","","The slam topic name to evaluate");
    config.arg<std::string>("cpu_log",slam_name+"_metric_cpu.txt","the result file");

    std::list<std::pair<FrameID,double> > _data;
    auto pub=messenger.advertise<Svar>("evaluate/cpu",0);

    CPUMetric metric;
    auto sub=messenger.subscribe(slam_name+"/curframe",[&](FramePtr fr){
        std::pair<FrameID,double> status(fr->id(),metric.usage());
        _data.push_back(status);
        pub.publish({{"id",fr->id()},{"usage",status.second}});
    });

    if(config.get("help",false)) return config.help();
    LOG(INFO)<<"Metric cpu ready.";

    Messenger::exec();
    if(_data.empty()) return 0;
    std::ofstream ofs(slam_name+"_metric_cpu.txt");
    for(std::pair<FrameID,double> it:_data){
        ofs<<std::get<0>(it)<<" "
          <<std::get<1>(it)<<"\n";
    }
    return 0;
}

GSLAM_REGISTER_APPLICATION(metric_cpu,metric_cpu);

