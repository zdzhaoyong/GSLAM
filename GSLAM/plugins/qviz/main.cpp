#include <QApplication>
#include "MainWindow.h"
#include <GSLAM/core/GSLAM.h>

using namespace GSLAM;

int run(Svar config){
    std::shared_ptr<GSLAM::MainWindow> mainWindow;

    svar.update(config);

    std::string dataset=svar.arg<std::string>("dataset","","The dataset going to play.");

    Subscriber subStop=messenger.subscribe("gslam.stop",0,[&mainWindow](bool stop){
        if(mainWindow) mainWindow->close();
    });

    if(svar.get<bool>("help",false)){
        svar.help();
        return 0;
    }

    Subscriber sub_dataset_status=messenger.subscribe("dataset/status",0,[&mainWindow](int status){
        if(mainWindow) mainWindow->datasetStatusUpdated(status);
    });

    Publisher  pub_gui=messenger.advertise<std::string>("dataset/control",0);
    Subscriber sub_gui=messenger.subscribe("visualize",0,&QVisualizer::process,this);
    auto _player=std::make_shared<GSLAM::DatasetPlayer>(GSLAM::Dataset(dataset));

    QApplication app(config.GetInt("argc"),
                     (char**)config.get<char**>("argv",nullptr));

    mainWindow=std::shared_ptr<MainWindow>(new GSLAM::MainWindow());
    mainWindow->pub_gui=pub_gui;
    mainWindow->show();

    if(!dataset.empty())
        mainWindow->slotStartDataset(dataset.c_str());

    return app.exec();
}

GSLAM_REGISTER_APPLICATION(qviz,run);
