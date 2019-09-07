// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: zd5945@126.com (Yong Zhao)
//
// Dataset is used to provide source frames for SLAM plugins

#ifndef GSLAM_DATASET_H
#define GSLAM_DATASET_H
#include "Svar.h"
#include "Registry.h"
#include "Messenger.h"
#include "Timer.h"
#include "Map.h"

namespace GSLAM{

class Dataset;
typedef std::shared_ptr<Dataset> DatasetPtr;

/**
  @brief The Dataset class

  Here is a little demo to implement an OpenCV monocular Dataset
  @code
#include <opencv2/opencv.hpp>
#include <GSLAM/core/GSLAM.h>

using namespace GSLAM;
using namespace std;

class DatasetOpenCVMono : public GSLAM::Dataset
{
public:
    DatasetOpenCVMono(){}
    virtual std::string type() const{return "DatasetOpenCVMono";}

    virtual bool open(const std::string& dataset){
        var.parseFile(dataset.c_str());

        if(!video.open(var.get<std::string>("video","")) return false;
        camera=Camera(var.get<std::vector<double>>("camera_paras",{}));
        if(!camera.isValid()) return false;
        _name=dataset;
        frameId=1;
        return true;
    }

    virtual bool isOpened(){return video.isOpened()&&camera.isValid();}

    virtual FramePtr grabFrame(){
        double timestamp=GSLAM::TicToc::timestamp();
        cv::Mat img;
        video>>img;
        return FramePtr(new FrameMono(frameId++,timestamp,gimg,camera,IMAGE_BGRA));
    }

    GSLAM::FrameID   frameId;
    cv::VideoCapture video;
    GSLAM::Camera    camera;
};

GSLAM_REGISTER_DATASET(DatasetOpenCVMono,cvmono)
  @endcode

  Compile this file as a shared library with name "gslamDB_cvmono", or compile this file along with the excutable file.
  It would be very easy to use the dataset:
  @code
  Dataset dataset("file_path.cvmono");
  if(dataset.isOpened())
     auto fr=dataset.grabFrame();// obtain a frame
  @endcode
*/


class Dataset : public GSLAM::GObject
{
public:
    typedef std::vector<std::string> StrVec;
    Dataset():_name("Untitled"){}
    Dataset(const std::string& dataset){open(dataset);}
    virtual ~Dataset(){}

    std::string         name() const{if(_impl) return _impl->type();else return _name;}
    virtual std::string type() const{if(_impl) return _impl->type();else return "Dataset";}
    virtual bool        isOpened(){if(_impl) return _impl->isOpened();else return false;}
    virtual FramePtr    grabFrame(){if(_impl) return _impl->grabFrame();else return FramePtr();}

    virtual bool        open(const std::string& dataset);
    virtual bool        isLive()const{
        if(_impl) return _impl->isLive();
        return false;
    }
protected:
    std::string _name;
    DatasetPtr  _impl;
};

inline bool Dataset::open(const std::string& dataset){
    DatasetPtr  impl;
    std::string extension;
    // The input path could be dataset configuration file or just a folder path
    size_t dotPosition=dataset.find_last_of('.');
    if(dotPosition!=std::string::npos)// call by extension
    {
        extension=dataset.substr(dotPosition+1);
    }
    if(extension.empty()) {
        LOG(ERROR)<<"Can't open dataset without extension.";
        return false;
    }

    Svar localImpl=svar["gslam"]["datasets"][extension];
    if(localImpl.isFunction()){
        impl=DatasetPtr(localImpl().castAs<Dataset*>());
    }

    if(!impl)
    {
        Svar plugin=Registry::load("gslamDB_"+extension);
        if(plugin.isUndefined()){
            LOG(ERROR)<<"Can't load plugin gslamDB_"<<extension<<".";
            return false;
        }
        Svar pluginImpl=plugin["gslam"]["datasets"][extension];
        if(!pluginImpl.isFunction())
        {
            LOG(ERROR)<<"Plugin gslamDB_"<<extension
                     <<" has no function gslam.datasets."<<extension;
            return false;
        }
        impl=DatasetPtr(pluginImpl().castAs<Dataset*>());
    }

    if(impl) {_impl=impl;return _impl->open(dataset);}
    return false;
}

}

#endif // VIDEOREADER_H
