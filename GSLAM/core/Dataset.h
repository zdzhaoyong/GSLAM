#ifndef GSLAM_DATASET_H
#define GSLAM_DATASET_H
#include "GSLAM.h"
#include "Svar.h"
#include "SharedLibrary.h"

namespace GSLAM{
/// create
class Dataset;
typedef SPtr<Dataset> (*funcCreateDataset)();
extern "C"
{
SPtr<Dataset> createDataset();
}

// A dataset configuration file : DatasetName.DatasetType --eg. Sequence1.kitti desk.tumrgbd
class Dataset : public GSLAM::GObject
{
    typedef SPtr<Dataset> Loader;
public:
    Dataset():_name("Untitled"){}
    virtual ~Dataset(){}

    virtual std::string type() const{if(_impl) return _impl->type();else return "Dataset";}
    std::string         name() const{if(_impl) return _impl->type();else return _name;}

    virtual bool open(const std::string& dataset){
        // The input path could be dataset configuration file or just a folder path
        int dotPosition=dataset.find_last_of('.');
        if(dotPosition!=std::string::npos)// call by extension
        {
            std::string ext=dataset.substr(dotPosition+1);
            Loader loader=loaders()[ext];
            if(loader)
            {
                if(loader->open(dataset))
                {
                    _impl=loader;
                    return true;
                }
            }
        }
        SvarWithType<Loader>::DataMap impls=loaders().get_data();
        for(SvarWithType<Loader >::DataIter it=impls.begin();it!=impls.end();it++)
        {
            Loader loader=it->second;
            if(loader->open(dataset))
            {
                _impl=loader;
                return true;
            }
        }
        return false;
    }

    virtual bool isOpened(){return _impl.get();}

    virtual FramePtr grabFrame(){if(_impl) return _impl->grabFrame();else return FramePtr();}

    static SvarWithType<Loader >& loaders()
    {
        static SPtr<SvarWithType<Loader> > globalLoaders(new SvarWithType<Loader>());
        return *globalLoaders;
    }

    static bool loadPlugin(const std::string& pluginPath)
    {
        typedef SPtr<SharedLibrary> SharedLibPtr;
        SharedLibPtr& lib=SvarWithType<SharedLibPtr>::instance()[pluginPath];
        if(lib) return lib->isLoaded();
        lib=SharedLibPtr(new SharedLibrary());
        if(!lib->load(pluginPath)) return false;
        funcCreateDataset factory=(funcCreateDataset)lib->getSymbol("createDataset");
        if(!factory) return false;
        Loader loader=factory();
        if(!loader) return false;
        loaders()[loader->type()]=loader;// loaded successfully
        return true;
    }

protected:
    std::string _name;
    Loader      _impl;
};

#define REGISTER_DATASET_(D)\
    class D##_Register{ \
    public: D##_Register(){D* d=new D();\
    Dataset::loaders()[d->type()]=SPtr<Dataset>(d);}\
}D##_inst;
#define REGISTER_DATASET(D) REGISTER_DATASET_(D)

}

#endif // VIDEOREADER_H
