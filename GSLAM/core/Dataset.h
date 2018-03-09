#ifndef GSLAM_DATASET_H
#define GSLAM_DATASET_H
#include "GSLAM.h"
#include "Svar.h"
#include "SharedLibrary.h"

namespace GSLAM{

#define REGISTER_DATASET(D,E) \
    extern "C" SPtr<GSLAM::Dataset> createDataset##E(){ return SPtr<GSLAM::Dataset>(new D());}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::DatasetFactory::instance()._ext2creator.insert(#E,createDataset##E);\
}}D##E##_instance;

/// create
class Dataset;
typedef SPtr<Dataset> DatasetPtr;
typedef SPtr<Dataset> (*funcCreateDataset)();

// A dataset configuration file : DatasetName.DatasetType --eg. Sequence1.kitti desk.tumrgbd
class Dataset : public GSLAM::GObject
{
public:
    typedef std::vector<std::string> StrVec;
    Dataset():_name("Untitled"){}
    virtual ~Dataset(){}

    std::string         name() const{if(_impl) return _impl->type();else return _name;}
    virtual std::string type() const{if(_impl) return _impl->type();else return "Dataset";}
    virtual bool        isOpened(){if(_impl) return _impl->isOpened();else return false;}
    virtual FramePtr    grabFrame(){if(_impl) return _impl->grabFrame();else return FramePtr();}

    virtual bool        open(const std::string& dataset);
protected:
    std::string _name;
    DatasetPtr  _impl;
};

class DatasetFactory
{
public:
    typedef SPtr<Dataset> DatasetPtr;

    static DatasetFactory& instance(){
        static SPtr<DatasetFactory> inst(new DatasetFactory());
        return *inst;
    }

    static DatasetPtr create(std::string dataset);

    SvarWithType<funcCreateDataset>        _ext2creator;
};

inline bool Dataset::open(const std::string& dataset){
    DatasetPtr impl=DatasetFactory::create(dataset);
    if(impl) {_impl=impl;return _impl->open(dataset);}
    return false;
}

inline DatasetPtr DatasetFactory::create(std::string dataset)
{
    std::string extension;
    // The input path could be dataset configuration file or just a folder path
    size_t dotPosition=dataset.find_last_of('.');
    if(dotPosition!=std::string::npos)// call by extension
    {
        extension=dataset.substr(dotPosition+1);
    }
    if(extension.empty()) return DatasetPtr();

    if(!instance()._ext2creator.exist(extension))
    {
        SharedLibraryPtr plugin=Registry::get("libgslamDB_"+extension);
        if(!plugin.get()) return DatasetPtr();
        funcCreateDataset createFunc=(funcCreateDataset)plugin->getSymbol("createDataset"+extension);
        if(createFunc) return createFunc();
    }

    if(!instance()._ext2creator.exist(extension)) return DatasetPtr();
    funcCreateDataset createFunc=instance()._ext2creator.get_var(extension,NULL);
    if(!createFunc) return DatasetPtr();

    return createFunc();
}

}

#endif // VIDEOREADER_H
