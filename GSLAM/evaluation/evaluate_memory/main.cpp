#include <GSLAM/core/GSLAM.h>
#include "MemoryMetric.h"

using namespace GSLAM;

int main(int argc,char** argv){
    Svar config;
    auto unParsed=config.parseMain(argc,argv);
    std::string slam_name=config.arg<std::string>("slam","","The slam topic name to evaluate");
    config.arg<std::string>("mem_log",slam_name+"_metric_memory.txt","the result file");

    std::list<std::tuple<FrameID,size_t,size_t> > _data;
    auto pub=messenger.advertise<std::tuple<FrameID,size_t,size_t> >("evaluate/memory",0);

    auto sub=messenger.subscribe(slam_name+"/curframe",[&](FramePtr fr){
        std::tuple<FrameID,size_t,size_t> status(fr->id(),
                                       MemoryMetric::instanceCPU().usage(),
                                       MemoryMetric::instanceCPU().count());
        _data.push_back(status);
        pub.publish(status);
    });

    MemoryMetric::instanceCPU().enable();

    std::vector<std::thread> threads;
    for(std::string& appname:unParsed){
        Svar run=svar["gslam"]["apps"][appname];

        if(!run.isFunction()){
            Svar app=Registry::load(appname);
            run=app["gslam"]["apps"][appname];
            if(!run.isFunction()) {
                LOG(WARNING)<<"Plugin "<<appname<<" does not have function \"run\".";
                continue;
            }

            Svar funcSetGlog=app["gslam"]["setGlobalLogSinks"];
            Svar funcSetMessenger=app["gslam"]["setGlobalMessenger"];

            if(funcSetGlog.isFunction()){
                funcSetGlog(getLogSinksGlobal());
            }

            if(funcSetMessenger.isFunction()){
                funcSetMessenger(messenger);
            }
            svar["gslam"]["apps"][appname]=run;
        }

        threads.push_back(std::thread([run,config](){run(config);}));
    }
    for(std::thread& th:threads) th.join();


    if(config.get("help",false)) return config.help();

    if(_data.empty()) return 0;
    std::ofstream ofs(slam_name+"_metric_memory.txt");
    for(std::tuple<FrameID,size_t,size_t> it:_data){
        ofs<<std::get<0>(it)<<" "
          <<std::get<1>(it)<<" "
         <<std::get<2>(it)<<"\n";
    }
    return 0;
}

