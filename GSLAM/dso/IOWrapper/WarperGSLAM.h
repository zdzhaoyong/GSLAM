#ifndef WARPERGSLAM_H
#define WARPERGSLAM_H
#include "Output3DWrapper.h"
#include "GSLAM/core/GSLAM.h"
#include "KeyFrameDisplay.h"

struct GraphConnection
{
    SPtr<dso::IOWrap::KeyFrameDisplay> from;
    SPtr<dso::IOWrap::KeyFrameDisplay> to;
    int fwdMarg, bwdMarg, fwdAct, bwdAct;
};

class WarperGSLAM : public dso::IOWrap::Output3DWrapper
{
public:
    WarperGSLAM(int width,int height);

    virtual ~WarperGSLAM();
    virtual void publishGraph(const std::map<long,Eigen::Vector2i> &connectivity);
    virtual void publishKeyframes(std::vector<dso::FrameHessian*> &frames, bool final, dso::CalibHessian* HCalib);
    virtual void publishCamPose(dso::FrameShell* frame, dso::CalibHessian* HCalib);

    virtual void pushLiveFrame(dso::FrameHessian* image);
    virtual void pushResidualImage(dso::MinimalImageB3* image);
    virtual void pushDepthImage(dso::MinimalImageB3* image);

    virtual void join();
    virtual void reset();

    void         draw();
    void drawConstraints();


    pi::Mutex      mutexImage;
    int            w,h;
    bool           videoChanged,kfImgChanged,resImgChanged;
    GSLAM::GImage  video,KFImg,ResImg;


    pi::Mutex      model3DMutex;
    SPtr<dso::IOWrap::KeyFrameDisplay> currentCam;
    std::vector<SPtr<dso::IOWrap::KeyFrameDisplay> > keyframes;
    std::map<int,SPtr<dso::IOWrap::KeyFrameDisplay> > keyframesByKFID;
    std::vector<GraphConnection> connections;
    std::vector<dso::Vec3f> allFramePoses;

    // render settings
    bool settings_showKFCameras;
    bool settings_showCurrentCamera;
    bool settings_showTrajectory;
    bool settings_showFullTrajectory;
    bool settings_showActiveConstraints;
    bool settings_showAllConstraints;

    float settings_scaledVarTH;
    float settings_absVarTH;
    int settings_pointCloudMode;
    float settings_minRelBS;
    int settings_sparsity;

    dso::SE3   lastPose;
};

#endif // WARPERGSLAM_H
