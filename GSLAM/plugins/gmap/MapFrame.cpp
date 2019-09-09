#include "MapFrame.h"
#include "MapPoint.h"

#include <GSLAM/core/Vocabulary.h>
#include <GSLAM/core/FileResource.h>

#ifndef NDEBUG
#define _DEBUG
#endif

MapFrame::MapFrame(const GSLAM::FrameID& id,const double& timestamp,
                   const GSLAM::GImage&  img,const std::string& imgFile,
                   const GSLAM::Camera&  camIn,const std::vector<double>& data,
                   int channalMask)
    : GSLAM::MapFrame(id,timestamp),_img(img),
      _imgFile(imgFile),_camIn(camIn),_gpshpyr(data),
      _medianDepth(-1),_imageChannel(channalMask)
{

}

bool   MapFrame::addObservation(const GSLAM::PointPtr& pt,size_t featId,bool add2Point)
{
    GSLAM::WriteMutex lock(_mutexPose);

    auto it=_obs.find(pt->id());
    if(it!=_obs.end())
    {
        DLOG(WARNING)<<("Observation already existed in MapFrame!");
        return false;    // already inserted
    }

    KeyPointData& kp=keyPoint(featId);
    if(kp._mapPoint.get())
    {
        DLOG(WARNING)<<("Observation already existed in MapFrame KeyPoints!");
        return false;// already matched
    }

    if(add2Point)
    {
        MapPoint* siftPt=(MapPoint*)pt.get();
        if(!siftPt->addObservation(id(),featId))
        {
            DLOG(WARNING)<<("Observation already existed in MapPoint!");
            return false;
        }
    }

    kp._mapPoint=pt;
    _obs.insert(std::make_pair(pt->id(),featId));
    return true;
}

bool   MapFrame::eraseObservation(const GSLAM::PointPtr& pt,bool erasePoint)
{
    GSLAM::WriteMutex lock(_mutexPose);

    auto it=_obs.find(pt->id());
    if(it==_obs.end())
    {
        DLOG(WARNING)<<("Failed to erase observation from MapFrame!");
        return false;    // no such observation
    }

    KeyPointData& kp=keyPoint(it->second);
    if(!kp._mapPoint){
        DLOG(WARNING)<<("Failed to erase observation from MapFrame!");
        return false;    // no such observation
    }

    if(erasePoint)
    {
        MapPoint* siftPt=(MapPoint*)pt.get();
        if(!siftPt->eraseObservation(id()))
        {
            DLOG(WARNING)<<("Failed to erase observation from MapPoint!");
            return false; //should never happen
        }
    }

    kp._mapPoint.reset();
    _obs.erase(it);
    return true;
}


bool  MapFrame::clearObservations(){
    for(auto it:_obs)
    {
        auto& mpt=keyPoint(it.second)._mapPoint;
        mpt->eraseObservation(id());
        mpt.reset();
    }
    _obs.clear();
    return true;
}

void  MapFrame::call(const std::string &command, void *arg)
{
    if("GetKeyPoint"==command)
    {
        std::pair<int,GSLAM::Point2d>* obs=(std::pair<int,GSLAM::Point2d>*)arg;
        GSLAM::Point2f pt=keyPoint(obs->first)._pt.pt;
        obs->second=GSLAM::Point2d(pt.x,pt.y);
    }
    else if("GetGPS"==command)
    {
        if((!arg)||_gpshpyr.empty()) return;
        std::vector<double>* dest=(std::vector<double>*)arg;
        *dest=_gpshpyr;
    }
    else if("GetImagePath"==command)
    {
        if(!arg) return;
        *(std::string*)arg=_imgFile;
    }
    else if("SetImagePath"==command){
        _imgFile=*(std::string*)arg;
    }
}

bool   MapFrame::getObservations(std::map<GSLAM::PointID,size_t>& obs)const
{
//    obs=_obs;
    for(int i=0;i<_keyPoints.size();++i){
        const KeyPointData& kp=_keyPoints[i];
        if(kp._mapPoint)
            obs.insert(std::make_pair(kp._mapPoint->id(),i));
    }
    return true;
}


double MapFrame::getMedianDepth()
{
    if(_medianDepth>0) return _medianDepth;
    std::map<GSLAM::PointID,size_t> observes;
    getObservations(observes);
    if(!observes.size()) return -1;

    GSLAM::SE3         w2c=getPose().inverse();
    std::vector<float> vDepths;
    for(auto obs:observes)
    {
        GSLAM::PointPtr pt=keyPoint(obs.second)._mapPoint;
        if(!pt.get()) continue;
        vDepths.push_back((w2c*pt->getPose()).z);
    }
    if(!vDepths.size()) return -1;
    sort(vDepths.begin(), vDepths.end());
    _medianDepth=vDepths[(vDepths.size()-1)/2];
    return _medianDepth;
}

bool   MapFrame::getBoWVector(GSLAM::BowVector &bowvec) const
{
//    if(_bowvec.size()) {bowvec=_bowvec;return true;}
//    GSLAM::Vocabulary& voc=svar[""];
//    if(voc.empty())
//    {
//        bool loaded=false;
//        while(true)
//        {
//            std::pair<char*,int> rc=GSLAM::FileResource::getResource(svar.GetString("SLAM.Vocabulary","sift_10_4_tf_l1.gbow"));
//            if(!rc.first) break;
//            std::stringbuf buf(std::string(rc.first,rc.second),std::ios::in);
//            std::istream   ist(&buf);
//            if(voc.load(ist)) loaded=true;
//            break;
//        }
//        if(!loaded)
//            if(voc.load(svar.GetString("SLAM.Vocabulary","sift_10_4_tf_l1.gbow"))) loaded=true;
//        if(!loaded) return false;
//    }


//    if(voc.empty()) return false;
//    if(_descriptors.empty()) return false;
//    voc.transform(_descriptors,_bowvec,_featvec,voc.getDepthLevels()-3);
//    bowvec=_bowvec;return true;
}

bool   MapFrame::getFeatureVector(GSLAM::FeatureVector &featvec) const
{
//    if(_featvec.size()) {featvec=_featvec;return true;}
//    GSLAM::Vocabulary& voc=GSLAM::SvarWithType<GSLAM::Vocabulary>::instance()["Vocabulary"];
//    if(voc.empty())
//    {
//        bool loaded=false;
//        while(true)
//        {
//            std::pair<char*,int> rc=GSLAM::FileResource::getResource(svar.GetString("SLAM.Vocabulary","sift_10_4_tf_l1.gbow"));
//            if(!rc.first) break;
//            std::stringbuf buf(std::string(rc.first,rc.second),std::ios::in);
//            std::istream   ist(&buf);
//            if(voc.load(ist)) loaded=true;
//            break;
//        }
//        if(!loaded)
//            if(voc.load(svar.GetString("SLAM.Vocabulary","sift_10_4_tf_l1.gbow"))) loaded=true;
//        if(!loaded) return false;
//    }

//    if(voc.empty()) return false;
//    if(_descriptors.empty()) return false;
//    voc.transform(_descriptors,_bowvec,_featvec,voc.getDepthLevels()-3);
//    featvec=_featvec;return true;
}

bool   MapFrame::setKeyPoints(const std::vector<GSLAM::KeyPoint>& kpts,
                                   const GSLAM::GImage& descriptors)
{
    _descriptors=descriptors;
    _keyPoints.resize(kpts.size());
    for(int i=0;i<kpts.size();i++)
    {
        KeyPointData& kp=_keyPoints[i];
        kp._pt=kpts[i];
        auto& pt=kp._pt.pt;
        kp._color=color(pt.x,pt.y);
        kp._ptUn=_camIn.UnProject(pt.x,pt.y);
    }

    // Assign Features to Grid Cells
    int nReserve = 0.5*_keyPoints.size()/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    float GridWidthInv =FRAME_GRID_COLS/(float)_camIn.width();
    float GridHeightInv=FRAME_GRID_ROWS/(float)_camIn.height();
    for(size_t i=0,iend=_keyPoints.size();i<iend;i++)
    {
        GSLAM::KeyPoint &kp = _keyPoints[i]._pt;

        int posX, posY;
        posX = round((kp.pt.x)*GridWidthInv);
        posY = round((kp.pt.y)*GridHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            continue;
        mGrid[posX][posY].push_back(i);
    }
    return true;
}

std::vector<size_t> MapFrame::getFeaturesInArea(const float &x, const float& y, const float &r,bool precisely)const
{
    using namespace std;
    vector<size_t> vIndices;
    GSLAM::Camera _camOut=_camIn;
    int mnMinX=0,mnMinY=0;

    float GridWidthInv =FRAME_GRID_COLS/(float)_camOut.width();
    float GridHeightInv=FRAME_GRID_ROWS/(float)_camOut.height();

    int nMinCellX = floor((x-r)*GridWidthInv);
    nMinCellX = max(0,nMinCellX);
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    int nMaxCellX = ceil((x-mnMinX+r)*GridWidthInv);
    nMaxCellX = min(FRAME_GRID_COLS-1,nMaxCellX);
    if(nMaxCellX<0)
        return vIndices;

    int nMinCellY = floor((y-mnMinY-r)*GridHeightInv);
    nMinCellY = max(0,nMinCellY);
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    int nMaxCellY = ceil((y-mnMinY+r)*GridHeightInv);
    nMaxCellY = min(FRAME_GRID_ROWS-1,nMaxCellY);
    if(nMaxCellY<0)
        return vIndices;

    vIndices.reserve(_keyPoints.size());
    float r2=r*r;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t idx:vCell)
            {
                if(precisely)
                {
                GSLAM::Point2f e=_keyPoints[idx]._pt.pt-GSLAM::Point2f(x,y);
                if(e.x*e.x+e.y*e.y<=r2)
                    vIndices.push_back(idx);
                }
                else vIndices.push_back(idx);
            }
        }
    }

    return vIndices;
}

GSLAM::ColorType MapFrame::color(int x,int y)
{
    if(_img.empty())
        return GSLAM::ColorType(255,255,255);
    if(imageChannels()&GSLAM::IMAGE_RGBA)
    {
        int idx=y*_img.cols+x;
        switch (_img.type()) {
        case GSLAM::GImageType<u_char,3>::Type:
            return _img.at<GSLAM::Point3ub>(idx);
            break;
        case GSLAM::GImageType<u_char,4>::Type:
            return *(GSLAM::Point3ub*)(_img.data+4*idx);
            break;
        case GSLAM::GImageType<u_char,1>::Type:
        {
            u_char uc=_img.data[idx];
            return GSLAM::Point3ub(uc,uc,uc);
        }
            break;
        default:
            return GSLAM::ColorType();
            break;
        }
    }
    else
    {
        int idx=y*_img.cols+x;
        switch (_img.type()) {
        case GSLAM::GImageType<u_char,3>::Type:
        {
            GSLAM::Point3ub c=_img.at<GSLAM::Point3ub>(idx);
            return GSLAM::Point3ub(c.z,c.y,c.x);
        }
            break;
        case GSLAM::GImageType<u_char,4>::Type:
        {
            GSLAM::Point3ub c=*(GSLAM::Point3ub*)(_img.data+4*idx);
            return GSLAM::Point3ub(c.z,c.y,c.x);
        }
            break;
        case GSLAM::GImageType<u_char,1>::Type:
        {
            u_char uc=_img.data[idx];
            return GSLAM::Point3ub(uc,uc,uc);
        }
            break;
        default:
            return GSLAM::ColorType();
            break;
        }
    }
    return GSLAM::ColorType(255,255,255);
}

GSLAM::SO3 PYR2Rotation(double pitch,double yaw,double roll)
{
    if(fabs(180-fabs(roll))<10) roll+=180;
    GSLAM::SO3 camera2IMU(-0.5,0.5,-0.5,0.5);
    GSLAM::SO3 imu2world=GSLAM::SO3::fromPitchYawRollAngle(-pitch,90.-yaw,roll);
    return imu2world*camera2IMU;
}

bool MapFrame::getPrioryPose(GSLAM::SIM3& sim3)// compute the priory pose of ECEF coordinate
{
    if(getGPSNum()==0||getIMUNum()==0) return false;// should have both GPS and IMU information
    GSLAM::Point3d lla,pyr;
    GSLAM::Point2d h;
    if(!getGPSLLA(lla)) return false;
    if(!getPitchYawRoll(pyr)) return false;
    bool heightValid=true;
    if(!getHeight2Ground(h)) heightValid=false;

    // Camera to local : east up north
    GSLAM::SIM3 camera2local(PYR2Rotation(pyr.x,pyr.y,pyr.z),
                             GSLAM::Point3d(0,0,0),
                             heightValid?h.x:getMedianDepth());

    // Local to ECEF
    GSLAM::SIM3 local2ECEF;
    local2ECEF.get_translation()=GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
    double D2R=3.1415925/180.;
    double lon=lla.x*D2R;
    double lat=lla.y*D2R;
    GSLAM::Point3d up(cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat));
    GSLAM::Point3d east(-sin(lon), cos(lon), 0);
    GSLAM::Point3d north=up.cross(east);
    double R[9]={east.x, north.x, up.x,
                 east.y, north.y, up.y,
                 east.z, north.z, up.z};
    local2ECEF.get_rotation().fromMatrix(R);

    // Camera to ECEF
    sim3= local2ECEF*camera2local;
    return true;
}
