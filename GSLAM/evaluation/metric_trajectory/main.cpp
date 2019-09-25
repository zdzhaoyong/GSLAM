#include <GSLAM/core/GSLAM.h>
using namespace GSLAM;

int metric_traj(Svar config){
    std::string slam_name=config.arg<std::string>("slam","","The slam topic name to evaluate");
    std::string vo_traj=config.arg<std::string>("traj_vo",slam_name+"_traj_vo.txt","the vo trajectory");
    std::string final_traj=config.arg<std::string>("traj_final",slam_name+"_traj_final.txt","the final trajectory");

    std::list<std::pair<double,SE3> > traj_vo;

    MapPtr map;
    auto sub0= messenger.subscribe(slam_name+"/map",[&](MapPtr mp){
        map=mp;
    });

    auto sub1=messenger.subscribe(slam_name+"/curframe",[&](FramePtr fr){
            traj_vo.push_back({fr->timestamp(),fr->getPose()});
    });

    if(config.get("help",false)) return config.help();
    LOG(INFO)<<"Metric trajectory ready.";
    Messenger::exec();

    if(traj_vo.size()){
        std::ofstream ofs(vo_traj.c_str());
        ofs.precision(15);
        for(auto it:traj_vo){
            ofs<<it.first<<" "
              <<it.second<<"\n";
        }
    }

    if(map&&map->frameNum()>0){
        std::ofstream ofs(final_traj.c_str());
        ofs.precision(15);
        FrameArray array=map->getFrames();
        std::sort(array.begin(),array.end(),[](FramePtr l,FramePtr r){
            return l->id()<r->id();
        });
        for(FramePtr fr: array){
            ofs<<fr->timestamp()<<" "<<fr->getPose()<<std::endl;
        }
    }
    return 0;
}

GSLAM_REGISTER_APPLICATION(metric_traj,metric_traj);

