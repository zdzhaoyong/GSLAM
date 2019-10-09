#include "SvarPy.h"
using namespace std;

namespace GSLAM{

REGISTER_SVAR_MODULE(gslam_builtin) {
    svar["__doc__"]="This is the python APIs for GSLAM "+string(GSLAM_VERSION_STR)
            +"(https://github.com/zdzhaoyong/GSLAM)";

    Class<Point2d>("Point2d")
            .construct<>()
            .construct<double,double>()
            .def("dot",&Point2d::dot)
            .def("norm",&Point2d::norm)
            .def("at",&Point2d::at)
            .def("__repr__",&Point2d::toString)
            .def("__str__",&Point2d::toString)
            .def("__add__",&Point2d::add)
            .def("__sub__",&Point2d::sub)
            .def("__mul__",&Point2d::mul)
            .def("__div__",&Point2d::div)
            .def_readwrite("x", &Point2d::x)
            .def_readwrite("y", &Point2d::y)
        ;

    Class<Point3d>("Point3d")
            .construct<>()
            .construct<double,double,double>()
            .def("dot",&Point3d::dot)
            .def("norm",&Point3d::norm)
            .def("at",&Point3d::at)
            .def("__repr__",&Point3d::toString)
            .def("__str__",&Point3d::toString)
            .def("__add__",&Point3d::add)
            .def("__sub__",&Point3d::sub)
            .def("__mul__",&Point3d::mul)
            .def("__div__",&Point3d::div)
            .def_readwrite("x", &Point3d::x)
            .def_readwrite("y", &Point3d::y)
            .def_readwrite("z", &Point3d::z);

    Class<Point3ub>("Point3ub")
            .construct<>()
            .construct<uchar,uchar,uchar>()
            .def("dot",&Point3ub::dot)
            .def("norm",&Point3ub::norm)
            .def("at",&Point3ub::at)
            .def("__repr__",&Point3ub::toString)
            .def("__str__",&Point3ub::toString)
            .def("__add__",&Point3ub::add)
            .def("__sub__",&Point3ub::sub)
            .def("__mul__",&Point3ub::mul)
            .def("__div__",&Point3ub::div)
            .def_readwrite("x", &Point3ub::x)
            .def_readwrite("y", &Point3ub::y)
            .def_readwrite("z", &Point3ub::z);

    Class<Camera>("Camera")
            .construct<>()
            .construct<std::vector<double> >()
            .def("CameraType",&Camera::CameraType)
            .def("info",&Camera::info)
            .def("isValid",&Camera::isValid)
            .def("width",&Camera::width)
            .def("height",&Camera::height)
            .def("getParameters",&Camera::getParameters)
            .def("estimatePinHoleCamera",&Camera::estimatePinHoleCamera)
            .def("Project",(Point2d (Camera::*)(const Point3d&)const) &Camera::Project,
                 "Project a point from camera coordinate to image coordinate")
            .def("UnProject",(Point3d (Camera::*)(const Point2d&)const) &Camera::UnProject,"");

    Class<SO3>("SO3")
            .construct<>()
            .construct<double,double,double,double>()
            .construct<const Point3d&,double>()
            .construct<const double*>()
            .def("log",&SO3::log)
            .def_static("exp",&SO3::exp<double>)
            .def("normalise",&SO3::normalise)
//            .def("getMatrix",&SO3::getMatrix)
            .def("__mul__",&SO3::mul)
            .def("trans",&SO3::trans)
            .def("inverse",&SO3::inverse)
            .def("__repr__",&SO3::toString)
            .def_readwrite("x", &SO3::x)
            .def_readwrite("y", &SO3::y)
            .def_readwrite("z", &SO3::z)
            .def_readwrite("w", &SO3::w)
            ;

    Class<SE3>("SE3")
            .construct<>()
            .construct<const SO3&,const Point3d&>()
            .def("log",&SE3::log)
            .def("inverse",&SE3::inverse)
            .def_static("exp",&SE3::exp<double>)
            .def("__mul__",&SE3::mul)
            .def("trans",&SE3::trans)
            .def("toString",&SE3::toString)
            .def("__repr__",&SE3::toString)
            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
            ;

    Class<SIM3>("SIM3")
            .construct<>()
            .construct<const SE3&,const double&>()
            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
            ;

    Class<GImage>("GImage")
            .construct<>()
            .construct<int,int,int,uchar*,bool>()
            .def("empty",&GImage::empty)
            .def_readonly("data",&GImage::data)
            .def("elemSize",&GImage::elemSize)
            .def("elemSize1",&GImage::elemSize1)
            .def("channels",&GImage::channels)
            .def("type",&GImage::type)
            .def("total",&GImage::total)
            .def("clone",&GImage::clone)
            .def("row",&GImage::row)
            .def("width",&GImage::getWidth)
            .def("height",&GImage::getHeight)
            .def("__repr__",[](const GImage& img){
                return to_string(img.cols)+"x"
                        +to_string(img.rows)+"x"+to_string(img.channels());})
            ;

    Class<TicToc>("TicToc")
            .construct<>()
            .def("timestamp",&TicToc::timestamp)
            .def("tic",&TicToc::Tic)
            .def("toc",&TicToc::Toc)
            ;

//    Class<Timer>("Timer")
//            .construct<bool>()
//            .def("enter",&Timer::enter)
//            .def("leave",&Timer::leave)
//            .def("disable",&Timer::disable)
//            .def("enable",&Timer::enable)
//            .def("getMeanTime",&Timer::getMeanTime)
//            .def("getStatsAsText",&Timer::getStatsAsText)
//            .def("dumpAllStats",&Timer::dumpAllStats)
//            ;

        Class<MapPoint>("MapPoint")
            .def("id",&MapPoint::id)
            .def("getPose",&MapPoint::getPose)
            .def("setPose",&MapPoint::setPose)
            .def("getNormal",&MapPoint::getNormal)
            .def("setNormal",&MapPoint::setNormal)
            .def("getColor",&MapPoint::setColor)
            .def("getDescriptor",&MapPoint::getDescriptor)
            .def("isPoseRelative",&MapPoint::isPoseRelative)
            .def("refKeyframeID",&MapPoint::refKeyframeID)
            .def("refKeyframe",&MapPoint::refKeyframe)
            .def("observationNum",&MapPoint::observationNum)
            .def("getObservations",(MapPointObsVec (MapPoint::*)()const) &MapPoint::getObservations)
            .def("eraseObservation",&MapPoint::eraseObservation)
            .def("clearObservation",&MapPoint::clearObservation)
            .def_property("pose", &MapPoint::getPose, &MapPoint::setPose)
            .def_property("normal", &MapPoint::getNormal, &MapPoint::setNormal)
            .def_property("color", &MapPoint::getColor, &MapPoint::setColor)
            .def_property("descriptor", &MapPoint::getDescriptor, &MapPoint::setDescriptor)
            ;

    Class<FrameConnection>("FrameConnection")
            .def("matchesNum",&FrameConnection::matchesNum)
            .def("getInformation",&FrameConnection::getInformation)
            .def("setMatches",&FrameConnection::setMatches)
            .def("setInformation",&FrameConnection::setInformation)
            .def("getMatches",(std::vector<std::pair<int,int> >(FrameConnection::*)())&FrameConnection::getMatches)
            ;

    Class<MapFrame>("MapFrame")
            .def("id",&MapFrame::id)
            .def("timestamp",&MapFrame::timestamp)
            .def("setPose",(void (MapFrame::*)(const SE3&))&MapFrame::setPose)
            .def("setPoseSim3",(void (MapFrame::*)(const SIM3&))&MapFrame::setPose)
            .def("getPose",(SE3(MapFrame::*)()const)&MapFrame::getPose)
            .def("getPoseScale",&MapFrame::getPoseScale)
            .def("cameraNum",&MapFrame::cameraNum)
            .def("getCameraPose",&MapFrame::getCameraPose)
            .def("imageChannels",&MapFrame::imageChannels)
            .def("getCamera",&MapFrame::getCamera)
            .def("getImage",&MapFrame::getImage)
            .def("setImage",&MapFrame::setImage)
            .def("setCamera",&MapFrame::setCamera)
            .def("getIMUNum",&MapFrame::getIMUNum)
            .def("getIMUPose",&MapFrame::getIMUPose)
            .def("getAcceleration",&MapFrame::getAcceleration)
            .def("getAngularVelocity",&MapFrame::getAngularVelocity)
            .def("getMagnetic",&MapFrame::getMagnetic)
            .def("getAccelerationNoise",&MapFrame::getAccelerationNoise)
            .def("getAngularVNoise",&MapFrame::getAngularVNoise)
            .def("getPitchYawRoll",&MapFrame::getPitchYawRoll)
            .def("getPYRSigma",&MapFrame::getPYRSigma)
            .def("getGPSNum",&MapFrame::getGPSNum)
            .def("getGPSPose",&MapFrame::getGPSPose)
            .def("getGPSLLA",&MapFrame::getGPSLLA)
            .def("getGPSLLASigma",&MapFrame::getGPSLLASigma)
            .def("getGPSECEF",&MapFrame::getGPSECEF)
            .def("getHeight2Ground",&MapFrame::getHeight2Ground)
            .def("getGPSECEF",&MapFrame::getGPSECEF)
            .def("getHeight2Ground",&MapFrame::getHeight2Ground)
            .def("keyPointNum",&MapFrame::keyPointNum)
            .def("setKeyPoints",&MapFrame::setKeyPoints)
            .def("getKeyPoints",(std::vector<KeyPoint>(MapFrame::*)() const)&MapFrame::getKeyPoints)
            .def("getKeyPointColor",&MapFrame::getKeyPointColor)
            .def("getKeyPointIDepthInfo",&MapFrame::getKeyPointIDepthInfo)
            .def("getKeyPointObserve",&MapFrame::getKeyPointObserve)
            .def("getDescriptor",&MapFrame::getDescriptor)
            .def("getBoWVector",(BowVector (MapFrame::*)()const)&MapFrame::getBoWVector)
            .def("getFeatureVector",(FeatureVector (MapFrame::*)()const)&MapFrame::getFeatureVector)
            .def("getFeaturesInArea",&MapFrame::getFeaturesInArea)
            .def("observationNum",&MapFrame::observationNum)
            .def("getObservations",(std::map<GSLAM::PointID,size_t>(MapFrame::*)()const)&MapFrame::getObservations)
            .def("addObservation",&MapFrame::addObservation)
            .def("eraseObservation",&MapFrame::eraseObservation)
            .def("clearObservations",&MapFrame::clearObservations)
            .def("getParent",&MapFrame::getParent)
            .def("getChild",&MapFrame::getChild)
            .def("getParents",(FrameConnectionMap (MapFrame::*)()const)&MapFrame::getParents)
            .def("getChildren",(FrameConnectionMap (MapFrame::*)()const)&MapFrame::getChildren)
            .def("addParent",&MapFrame::addParent)
            .def("addChildren",&MapFrame::addChildren)
            .def("eraseParent",&MapFrame::eraseParent)
            .def("eraseChild",&MapFrame::eraseChild)
            .def("clearParents",&MapFrame::clearParents)
            .def("clearChildren",&MapFrame::clearChildren)
            .def("getMedianDepth",&MapFrame::getMedianDepth)
            .def("channelTypeString",&MapFrame::channelTypeString)
            .def("channelString",&MapFrame::channelString)
            ;

   Class<Dataset>("Dataset")
            .construct<>()
            .construct<const std::string&>()
            .def("name",&Dataset::name)
            .def("type",&Dataset::type)
            .def("isOpened",&Dataset::isOpened)
            .def("grabFrame",&Dataset::grabFrame)
            .def("open",&Dataset::open)
            ;

    Class<Map>("Map")
            .def("insertMapPoint",&Map::insertMapPoint)
            .def("insertMapFrame",&Map::insertMapFrame)
            .def("eraseMapPoint",&Map::eraseMapPoint)
            .def("eraseMapFrame",&Map::eraseMapFrame)
            .def("clear",&Map::clear)
            .def("frameNum",&Map::frameNum)
            .def("pointNum",&Map::pointNum)
            .def("getFrame",&Map::getFrame)
            .def("getPoint",&Map::getPoint)
            .def("getFrames",(FrameArray(Map::*)()const)&Map::getFrames)
            .def("getPoints",(PointArray(Map::*)()const)&Map::getPoints)
            .def("setLoopDetector",&Map::setLoopDetector)
            .def("getLoopDetector",&Map::getLoopDetector)
            .def("obtainCandidates",(LoopCandidates(Map::*)(const FramePtr&))&Map::obtainCandidates)
            .def("save",&Map::save)
            .def("load",&Map::load)
            .def("getPid",&Map::getPid)
            .def("getFid",&Map::getFid)
            ;

//    py::class_<HashMap,Map,SPtr<HashMap> >(m,"HashMap")
//            .construct<>())
//            ;

    Class<Vocabulary>("Vocabulary")
            .construct<const std::string &>()
            .def_static("create",&Vocabulary::create)
            .def("save",&Vocabulary::save)
            .def("load",(bool(Vocabulary::*)(const std::string &))&Vocabulary::load)
            .def("size",&Vocabulary::size)
            .def("empty",&Vocabulary::empty)
            .def("clear",&Vocabulary::clear)
            .def("transformImage",(void (Vocabulary::*)(const TinyMat&,
                 BowVector &, FeatureVector &, int)const)&Vocabulary::transform)
            .def("transformFeature",(void (Vocabulary::*)(const TinyMat &,
                 WordId &, WordValue &, NodeId*, int) const)&Vocabulary::transform)
            .def("getBranchingFactor",&Vocabulary::getBranchingFactor)
            .def("getDepthLevels",&Vocabulary::getDepthLevels)
            .def("getWord",&Vocabulary::getWord)
            .def("getWordWeight",&Vocabulary::getWordWeight)
            .def("getWeightingType",&Vocabulary::getWeightingType)
            .def("getScoringType",&Vocabulary::getScoringType)
            .def("setWeightingType",&Vocabulary::setWeightingType)
            .def("getDescritorSize",&Vocabulary::getDescritorSize)
            .def("getDescritorType",&Vocabulary::getDescritorType)
            .def_static("meanValue",&Vocabulary::meanValue)
            .def_static("distance",&Vocabulary::distance)
            ;

    Class<FileResource>("FileResource")
            .def_static("toHex",&FileResource::toHex)
            .def_static("exportResourceFile",&FileResource::exportResourceFile)
            .def_static("Register",&FileResource::Register)
            .def_static("saveResource2File",&FileResource::saveResource2File)
            ;

    Class<Undistorter>("Undistorter")
            .construct<Camera, Camera>()
//            .def("undistort",&Undistorter::undistort)
//            .def("undistortFast",&Undistorter::undistortFast)
            .def("cameraIn",&Undistorter::cameraIn)
            .def("cameraOut",&Undistorter::cameraOut)
            .def("prepareReMap",&Undistorter::prepareReMap)
            .def("valid",&Undistorter::valid)
            ;

    Class<Messenger>("Messenger")
            .construct<>()
            .def_static("instance",&Messenger::instance)
            .def("getPublishers",&Messenger::getPublishers)
            .def("getSubscribers",&Messenger::getSubscribers)
            .def("introduction",&Messenger::introduction)
            .def("advertise",[](Messenger msg,const std::string& topic,int queue_size){
      return msg.advertise<Svar>(topic,queue_size);
    })
    .def("subscribe",[](Messenger msger,
         const std::string& topic, int queue_size,
         Svar callback){
      return msger.subscribe(topic,queue_size,[callback](Svar msg){callback(msg);});
    })
    .def("publish",[](Messenger* msger,std::string topic,Svar msg){return msger->publish(topic,msg);});
    // FIXME: cast to 'const Svar&'

    Class<Publisher>("Publisher")
            .def("shutdown",&Publisher::shutdown)
            .def("getTopic",&Publisher::getTopic)
            .def("getTypeName",&Publisher::getTypeName)
            .def("getNumSubscribers",&Publisher::getNumSubscribers)
            .def("publish",[](Publisher* pubptr,Svar msg){return pubptr->publish(msg);});

    Class<Subscriber>("Subscriber")
            .def("shutdown",&Subscriber::shutdown)
            .def("getTopic",&Subscriber::getTopic)
            .def("getTypeName",&Subscriber::getTypeName)
            .def("getNumPublishers",&Subscriber::getNumPublishers);

    svar["messenger"]=Messenger::instance();

}


}
