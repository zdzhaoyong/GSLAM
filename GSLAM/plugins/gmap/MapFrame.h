#ifndef MAPFRAME_H
#define MAPFRAME_H
#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/GPS.h>

#define FRAME_GRID_ROWS 32
#define FRAME_GRID_COLS 32

typedef unsigned int uint;

struct KeyPointData
{
    GSLAM::KeyPoint         _pt;
    GSLAM::Point3d          _ptUn;
    GSLAM::ColorType        _color;
    int                     _flags;
    std::shared_ptr<GSLAM::MapPoint>   _mapPoint;
};

class MapFrame : public GSLAM::MapFrame
{
public:
    MapFrame(const GSLAM::FrameID& id,const double& timestamp,
             const GSLAM::GImage&  img,const std::string& imgFile,
             const GSLAM::Camera&  camIn,const std::vector<double>& data,
             int channalMask=GSLAM::IMAGE_RGBA);
    virtual ~MapFrame(){}

    virtual std::string type()const{return "DIYFrame";}
    virtual void        call(const std::string &command, void *arg);

    virtual int            cameraNum()const{return 1;}
    virtual int            imageChannels(int idx=0) const{return _imageChannel;}
    virtual GSLAM::GImage  getImage(int idx=0,int channalMask=GSLAM::IMAGE_UNDEFINED){return _img;}
    virtual GSLAM::Camera  getCamera(int idx=0){return _camIn;}
    virtual bool           setImage(const GSLAM::GImage& img,int idx=0,int channalMask=GSLAM::IMAGE_UNDEFINED){_img=img;return true;}

    // When the frame contains IMUs or GPSs
    virtual int     getIMUNum()const{return (_gpshpyr.size()==11||_gpshpyr.size()==12||_gpshpyr.size()==14)?1:0;}
    virtual bool    getAcceleration(GSLAM::Point3d& acc,int idx=0)const{return false;}        // m/s^2
    virtual bool    getAngularVelocity(GSLAM::Point3d& angularV,int idx=0)const{return false;}// rad/s
    virtual bool    getMagnetic(GSLAM::Point3d& mag,int idx=0)const{return false;}            // gauss
    virtual bool    getAccelerationNoise(GSLAM::Point3d& accN,int idx=0)const{return false;}
    virtual bool    getAngularVNoise(GSLAM::Point3d& angularVN,int idx=0)const{return false;}
    virtual bool    getPitchYawRoll(GSLAM::Point3d& pyr,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyr=GSLAM::Point3d(_gpshpyr[5],_gpshpyr[6],_gpshpyr[7]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]<20) {pyr=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyr=GSLAM::Point3d(_gpshpyr[6],_gpshpyr[7],_gpshpyr[8]);return true;}
        return false;
    }     // in rad
    virtual bool    getPYRSigma(GSLAM::Point3d& pyrSigma,int idx=0)const{
        if(_gpshpyr.size()==11) {pyrSigma=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==14) {pyrSigma=GSLAM::Point3d(_gpshpyr[11],_gpshpyr[12],_gpshpyr[13]);return true;}
        else if(_gpshpyr.size()==12) {pyrSigma=GSLAM::Point3d(_gpshpyr[9],_gpshpyr[10],_gpshpyr[11]);return true;}
        return false;
    }    // in rad

    virtual int     getGPSNum()const{return (_gpshpyr.size()>=6&&_gpshpyr[3]<10)?1:0;}
    virtual bool    getGPSLLA(GSLAM::Point3d& LonLatAlt,int idx=0)const{
        if(getGPSNum()==0) return false;
        LonLatAlt=GSLAM::Point3d(_gpshpyr[0],_gpshpyr[1],_gpshpyr[2]);
        return _gpshpyr[3]<100;
    }        // WGS84 [longtitude latitude altitude]
    virtual bool    getGPSLLASigma(GSLAM::Point3d& llaSigma,int idx=0)const{
        if(_gpshpyr.size()>=6||_gpshpyr.size()==8||_gpshpyr.size()==12||_gpshpyr.size()==14)
        {llaSigma=GSLAM::Point3d(_gpshpyr[3],_gpshpyr[4],_gpshpyr[5]);return true;}
        else if(_gpshpyr.size()==7) {llaSigma=GSLAM::Point3d(_gpshpyr[3],_gpshpyr[3],_gpshpyr[4]);return true;}
        return false;
    }    // meter
    virtual bool    getGPSECEF(GSLAM::Point3d& xyz,int idx=0)const{
        GSLAM::Point3d lla;
        if(!getGPSLLA(lla)) return false;
        xyz=GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
        return true;
    }             // meter
    virtual bool    getHeight2Ground(GSLAM::Point2d& height,int idx=0)const{
        if(_gpshpyr.size()==14||_gpshpyr.size()==8){height=GSLAM::Point2d(_gpshpyr[6],_gpshpyr[7]);return _gpshpyr[7]<100;}
        return false;
    }    // height against ground

    virtual int    keyPointNum()const{return _keyPoints.size();}
    virtual bool   getKeyPoint(int idx,GSLAM::Point2f& pt)const
    {auto ptcv=_keyPoints[idx]._pt.pt;pt=GSLAM::Point2f(ptcv.x,ptcv.y);return true;}
    virtual bool   getKeyPoint(int idx,GSLAM::KeyPoint& pt)const
    {pt=_keyPoints[idx]._pt;return true;}

    virtual bool   getKeyPoints(std::vector<GSLAM::Point2f>& keypoints)const{
        keypoints.reserve(_keyPoints.size());
        for (auto ptc :_keyPoints) {
            keypoints.push_back(ptc._pt.pt);
        }
        return true;
    }

    virtual std::vector<GSLAM::KeyPoint>  getKeyPoints() const
    {
        std::vector<GSLAM::KeyPoint> r;
        r.resize(_keyPoints.size());
        for (int i = 0; i <_keyPoints.size() ; ++i)
        {
            getKeyPoint(i,r[i]);
        }
        return r;
    }

    virtual bool   getKeyPointColor(int idx,GSLAM::ColorType& color){color=_keyPoints[idx]._color;return true;}
    virtual GSLAM::PointID getKeyPointObserve(int idx){
        GSLAM::PointPtr& pt=_keyPoints[idx]._mapPoint;
        if(pt) return pt->id();
        else   return 0;
    }
    virtual bool   getKeyPointIDepthInfo(int idx, GSLAM::Point2d &idepth){
        GSLAM::PointPtr& pt=_keyPoints[idx]._mapPoint;
        if(!pt) return false;
        GSLAM::Point3d p3d=getPose().inverse()*pt->getPose();
        if(p3d.z<0.1) return false;
        idepth=GSLAM::Point2d(1./p3d.z,-1);
        return true;
    }
    virtual GSLAM::GImage getDescriptor(int idx)const{
        if(idx>=0) return _descriptors.row(idx);
        else return _descriptors;
    }

    virtual bool   setKeyPoints(const std::vector<GSLAM::KeyPoint>& keypoints,
                                 const GSLAM::GImage& descriptors=GSLAM::GImage());

    virtual bool   getBoWVector(GSLAM::BowVector &bowvec) const;
    virtual bool   getFeatureVector(GSLAM::FeatureVector &featvec) const;

    virtual int    observationNum()const{return _obs.size();}
    virtual bool   getObservations(std::map<GSLAM::PointID,size_t>& obs)const;
    virtual bool   addObservation(const GSLAM::PointPtr& pt,size_t featId,bool add2Point=false);
    virtual bool   eraseObservation(const GSLAM::PointPtr& pt,bool erasePoint=false);
    virtual bool   clearObservations();

    virtual GSLAM::FrameConnectionPtr getParent(GSLAM::FrameID parentId)const{
        auto it=_parents.find(parentId);
        if(it==_parents.end()) return GSLAM::FrameConnectionPtr();
        else return it->second;
    }
    virtual GSLAM::FrameConnectionPtr getChild(GSLAM::FrameID childId)const{
        auto it=_children.find(childId);
        if(it==_children.end()) return GSLAM::FrameConnectionPtr();
        else return it->second;
    }
    virtual bool    getParents(std::map<GSLAM::FrameID,GSLAM::FrameConnectionPtr >& parents)const{parents=_parents;return true;}
    virtual bool    getChildren(std::map<GSLAM::FrameID,GSLAM::FrameConnectionPtr >& children)const{children=_children;return true;}
    virtual bool    addParent(GSLAM::FrameID parentId,const GSLAM::FrameConnectionPtr& parent){_parents[parentId]=parent;return true;}
    virtual bool    addChildren(GSLAM::FrameID childId,const GSLAM::FrameConnectionPtr& child){_children[childId]=child;return true;}
    virtual bool    eraseParent(GSLAM::FrameID parentId){return _parents.erase(parentId);}
    virtual bool    eraseChild(GSLAM::FrameID  childId){return _children.erase(childId);}
    virtual bool    clearParents(){_parents.clear();return true;}
    virtual bool    clearChildren(){_children.clear();return true;}

    virtual double getMedianDepth();
    virtual std::vector<size_t> getFeaturesInArea(const float& x,const float& y,
                                                  const float& r,bool precisely=true)const;

    GSLAM::ColorType color(int x,int y);

    KeyPointData&  keyPoint(uint idx){return _keyPoints[idx];}
    std::vector<KeyPointData>& keyPoints(){return _keyPoints;}

    const std::vector<std::size_t>& getGrid(int x,int y){return mGrid[x][y];}

    bool getPrioryPose(GSLAM::SIM3& sim3);

private:
    GSLAM::GImage                       _img;
    int                                 _imageChannel;
    std::string                         _imgFile; // the image are cached to the disk instead of memory
    GSLAM::Camera                       _camIn; // _camIn: corrospond to real cam,_camOut: Pinhole camera
    std::vector<double>                 _gpshpyr;

    GSLAM::GImage                       _descriptors;
    std::vector<KeyPointData>           _keyPoints;

    std::map<GSLAM::PointID,size_t>     _obs;

    // Mapframe data
    int                                 _flags;
    std::map<GSLAM::FrameID,GSLAM::FrameConnectionPtr> _parents,_children;

    mutable GSLAM::BowVector            _bowvec;
    mutable GSLAM::FeatureVector        _featvec;

    // cache things, can be rebuild
    std::vector<std::size_t>            mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    double                              _medianDepth;
};

#endif // MAPFRAME_H
