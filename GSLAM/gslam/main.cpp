#include <GSLAM/core/GSLAM.h>
#include <list>
#include <sstream>
#include <regex>
#include <signal.h>
#include "filesystem.hpp"

using namespace std;
using namespace GSLAM;
using namespace ghc::filesystem;

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
            svar["gslam"]["apps"][appname]=run;
        }

        threads.push_back(std::thread([run](){run(svar);}));
    }

    if(threads.empty())
    {
        if(svar.get("complete_function_request",false)){
            std::cout<<svar.helpInfo();
            regex is_gslam_app("^(?:|lib)?gslam_([a-zA-Z\\d_]+).(?:|so|dll|dylib)$");
            for(auto folder:Registry::instance().paths())
            {
                if(!exists(path(folder))) continue;
                for(auto fileit:directory_iterator(folder))
                {
                    smatch result;
                    std::string filename = fileit.path().filename();
                    if(std::regex_match(filename,result,is_gslam_app))
                    {
                        if(result.size()<2) continue;
                        std::cout<<" "<<result.str(1);
                    }
                }
            }
            return 0;
        }
        std::cerr<<"Usage:\n  gslam [app1] -arg1 value1 [app2] -arg2 value2";
        return 0;
    }

    std::promise<bool> stopSig;
    Subscriber subStop=messenger.subscribe("messenger/stop",0,[&stopSig](bool b){
        stopSig.set_value(true);
    });
    signal(SIGINT, [](int sig){
        messenger.publish("messenger/stop",true);
    });
//    stopSig.get_future().wait();
    for(std::thread& th:threads) th.join();
    return 0;
}


