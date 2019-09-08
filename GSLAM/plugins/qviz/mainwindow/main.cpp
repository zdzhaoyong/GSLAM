#include <QApplication>
#include <QDir>
#include "MainWindow.h"
#include <GSLAM/core/GSLAM.h>

using namespace GSLAM;

int run(Svar config){
    GSLAM::MainWindow* mainWindow=nullptr;

    std::string dataset=config.arg<std::string>("dataset","","The dataset going to play.");

    Subscriber subStop=messenger.subscribe("gslam.stop",0,[&mainWindow](bool stop){
        if(mainWindow) mainWindow->close();
    });

    Subscriber sub_dataset_status=messenger.subscribe("dataset/status",[&mainWindow](int status){
        DLOG(INFO)<<"Dataset status updated to "<<status;
        if(mainWindow) mainWindow->datasetStatusUpdated(status);
    });

    Subscriber sub_panel=messenger.subscribe("qviz/panels",[&mainWindow](QWidget* panel)
    {
        if(!mainWindow) return;
        mainWindow->addPanel(panel);
    });

    if(config.get("help",false)){
        Publisher  pub_gui=messenger.advertise<std::string>("dataset/control",0);
        Publisher  pub_draw=messenger.advertise<Svar>("qviz/draw");

        config["__usage__"]=messenger.introduction();
        return config.help();
    }

    QApplication app(config.GetInt("argc"),
                     config.get<char**>("argv",nullptr));

    mainWindow=new GSLAM::MainWindow(nullptr,config);
    mainWindow->show();

    return app.exec();
}

GSLAM_REGISTER_APPLICATION(qviz,run);
