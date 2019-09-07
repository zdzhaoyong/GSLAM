#include <GSLAM/core/GSLAM.h>
#include <list>
#include <sstream>
#include <signal.h>

using namespace std;
using namespace GSLAM;

int main(int argc,char** argv)
{
    auto unParsed=svar.ParseMain(argc,argv);

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
        }

        threads.push_back(std::thread([run](){run(svar);}));
    }

    if(threads.empty())
    {
        if(svar.get("complete_function_request",false)){
            std::cout<<svar.helpInfo()+" doc qviz resource tests";
            return 0;
        }
        std::cerr<<"Usage:\n  gslam [app1] -arg1 value1 [app2] -arg2 value2";
        return 0;
    }

    std::promise<bool> stopSig;
    Subscriber subStop=messenger.subscribe("gslam.stop",0,[&stopSig](bool b){
        stopSig.set_value(true);
    });
    signal(SIGINT, [](int sig){
        messenger.publish("gslam.stop",true);
    });
//    stopSig.get_future().wait();
    for(std::thread& th:threads) th.join();
    return 0;
}


