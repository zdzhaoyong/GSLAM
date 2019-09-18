#include "MapHash.h"

using namespace GSLAM;

int run(Svar config){
    std::string in =config.arg<std::string>("in","","the map to load");
    std::string out=config.arg<std::string>("out","","the map file to save");
    std::string map=config.arg<std::string>("map","map","the map topic to publish or subscribe");

    Publisher   pub=messenger.advertise<MapPtr>(map);

    Subscriber  sub=messenger.subscribe(map,[out](MapPtr mp){
            if(out.size()) mp->save(out);
    });

    Subscriber  subOpen=messenger.subscribe("qviz/open",1,[&](std::string file){
        if(file.find(".gmap")==std::string::npos) return;
        MapPtr mapHash(new MapHash());
        mapHash->load(file);
        pub.publish(Svar::create(mapHash));
    });

    if(config.get("help",false)){
        config["__usage__"]=messenger.introduction();
        return config.help();
    }

    auto sub_request=messenger.subscribe("qviz/ready",[&](bool){
        if(in.size()){
            MapPtr mapHash(new MapHash());
            mapHash->load(in);
            pub.publish(Svar::create(mapHash));
        }
    });

    return Messenger::exec();
}

GSLAM_REGISTER_APPLICATION(gmap,run);
