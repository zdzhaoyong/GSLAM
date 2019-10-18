#include <GSLAM/core/GSLAM.h>

using namespace GSLAM;


int run_topic(Svar config){
    std::string echo      =config.arg<std::string>("echo","",R"(topics to echo, eg: test or use json '["test","topic2]')");
    std::string remap     =config.arg<std::string>("remap","",R"(use json map: '{"old":"new","old_name":"new_name"}')");

    if(config.get("help",false)) return config.help();

    Svar subs;
    Svar remapVar;

    if(echo.size()){
        std::vector<std::string> topics;
        try{
            if(Svar::json(echo).isArray())
                topics=Svar::json(echo).castAs<std::vector<std::string>>();
        }
        catch(SvarExeption){
            topics.push_back(echo);
        }
        for(std::string topic:topics)
            subs[topic]=messenger.subscribe(topic,[topic](Svar msg){
               std::cerr<<topic<<":"<<msg<<std::endl;
        });
    }

    if(remap.size()){
        auto mp=Svar::json(remap).castAs<std::map<std::string,std::string>>();
        for(auto p:mp){
            GSLAM::Publisher pub=messenger.advertise<Svar>(p.second);
            remapVar[p.first]=messenger.subscribe(p.first,[pub](Svar msg){pub.publish(msg);});
        }
    }

    if(echo.empty()&&remap.empty()) return 0;
    return Messenger::exec();
}

GSLAM_REGISTER_APPLICATION(topic,run_topic);
