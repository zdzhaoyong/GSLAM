#include <GSLAM/core/GSLAM.h>
using namespace GSLAM;

int metric_time(Svar config){
    std::string slam_name=config.arg<std::string>("slam","","The slam topic name to evaluate");
    config.arg<std::string>("time_log",slam_name+"_metric_time.txt","the result file");

    std::map<FrameID,double> _data;
    auto pub=messenger.advertise<Svar>("evaluate/time",0);

    auto sub0= messenger.subscribe("dataset/frame",[&](FramePtr fr){
        if(!fr->cameraNum()) return;
        _data[fr->id()]=GSLAM::TicToc::timestamp();
    });

    auto sub1=messenger.subscribe(slam_name+"/curframe",[&](FramePtr fr){
        if(!fr->cameraNum()) return;
        double& t=_data[fr->id()];
        t=GSLAM::TicToc::timestamp()-t;
        pub.publish({{"id",fr->id()},{"time",t}});
    });

    if(config.get("help",false)) return config.help();

    LOG(INFO)<<"Metric time ready.";
    Messenger::exec();
    if(_data.empty()) return 0;
    std::ofstream ofs(slam_name+"_metric_time.txt");
    for(std::tuple<FrameID,float> it:_data){
        ofs<<std::get<0>(it)<<" "
          <<std::get<1>(it)<<"\n";
    }
    return 0;
}

GSLAM_REGISTER_APPLICATION(metric_time,metric_time);

