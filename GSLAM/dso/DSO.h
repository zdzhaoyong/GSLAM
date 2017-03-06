#ifndef DSO_H
#define DSO_H
#include <iostream>
#include <GSLAM/core/GSLAM.h>

namespace dso
{
    class FullSystem;
    class Undistort;
}

class WarperGSLAM;
class ImageFolderReader;

namespace GSLAM {


class DSO: public SLAM
{
public:
    DSO();
    virtual ~DSO();

    virtual std::string type()const{return "DSO";}
    virtual bool valid()const;

    virtual bool track(FramePtr& frame);

    virtual void draw();

    virtual void call(const std::string& command,void* arg=NULL);

    SPtr<dso::FullSystem>   system;
    SPtr<WarperGSLAM>       warper;

    SPtr<ImageFolderReader> reader;

    bool                    shouldUndistort;
};

}
#endif // DSO_H
