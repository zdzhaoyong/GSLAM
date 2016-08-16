#ifndef VIDEOREADER_H
#define VIDEOREADER_H
#include <GSLAM/core/GSLAM.h>

class VideoReader : public GSLAM::GObject
{
public:
    VideoReader(const std::string& name="");

    virtual ~VideoReader(){}

    virtual std::string type() const;

    virtual GSLAM::FramePtr grabFrame();

    virtual bool isOpened();

private:
    SPtr<VideoReader> impl;
};

#endif // VIDEOREADER_H
