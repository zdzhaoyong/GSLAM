#include "SvarPy.h"
using namespace std;

// The old Python Thread Local Storage (TLS) API is deprecated in Python 3.7 in favor of the new
// Thread Specific Storage (TSS) API.
#if PY_VERSION_HEX >= 0x03070000
#    define PYBIND11_TLS_KEY_INIT(var) Py_tss_t *var = nullptr
#    define PYBIND11_TLS_GET_VALUE(key) PyThread_tss_get((key))
#    define PYBIND11_TLS_REPLACE_VALUE(key, value) PyThread_tss_set((key), (tstate))
#    define PYBIND11_TLS_DELETE_VALUE(key) PyThread_tss_set((key), nullptr)
#else
    // Usually an int but a long on Cygwin64 with Python 3.x
#    define PYBIND11_TLS_KEY_INIT(var) decltype(PyThread_create_key()) var = 0
#    define PYBIND11_TLS_GET_VALUE(key) PyThread_get_key_value((key))
#    if PY_MAJOR_VERSION < 3
#        define PYBIND11_TLS_DELETE_VALUE(key)                               \
             PyThread_delete_key_value(key)
#        define PYBIND11_TLS_REPLACE_VALUE(key, value)                       \
             do {                                                            \
                 PyThread_delete_key_value((key));                           \
                 PyThread_set_key_value((key), (value));                     \
             } while (false)
#    else
#        define PYBIND11_TLS_DELETE_VALUE(key)                               \
             PyThread_set_key_value((key), nullptr)
#        define PYBIND11_TLS_REPLACE_VALUE(key, value)                       \
             PyThread_set_key_value((key), (value))
#    endif
#endif

namespace GSLAM{


//class gil_scoped_acquire {
//public:

//  inline PyThreadState *get_thread_state_unchecked() {
//  #if defined(PYPY_VERSION)
//      return PyThreadState_GET();
//  #elif PY_VERSION_HEX < 0x03000000
//      return _PyThreadState_Current;
//  #elif PY_VERSION_HEX < 0x03050000
//      return (PyThreadState*) _Py_atomic_load_relaxed(&_PyThreadState_Current);
//  #elif PY_VERSION_HEX < 0x03050200
//      return (PyThreadState*) _PyThreadState_Current.value;
//  #else
//      return _PyThreadState_UncheckedGet();
//  #endif
//  }

//    gil_scoped_acquire() {
//      static PYBIND11_TLS_KEY_INIT(tstate);
//      static PyInterpreterState *istate = nullptr;
//        tstate = (PyThreadState *) PYBIND11_TLS_GET_VALUE(tstate);

//        if (!tstate) {
//            tstate = PyThreadState_New(istate);
//            #if !defined(NDEBUG)
//                if (!tstate)
//                    LOG(FATAL)<<("scoped_acquire: could not create thread state!");
//            #endif
//            tstate->gilstate_counter = 0;
//            PYBIND11_TLS_REPLACE_VALUE(tstate, tstate);
//        } else {
//            release = get_thread_state_unchecked() != tstate;
//        }

//        if (release) {
//            /* Work around an annoying assertion in PyThreadState_Swap */
//            #if defined(Py_DEBUG)
//                PyInterpreterState *interp = tstate->interp;
//                tstate->interp = nullptr;
//            #endif
//            PyEval_AcquireThread(tstate);
//            #if defined(Py_DEBUG)
//                tstate->interp = interp;
//            #endif
//        }

//        inc_ref();
//    }

//    void inc_ref() {
//        ++tstate->gilstate_counter;
//    }

//    void dec_ref() {
//        --tstate->gilstate_counter;
//        if (tstate->gilstate_counter == 0) {
//            #if !defined(NDEBUG)
//                if (!release)
//                    LOG(FATAL)<<("scoped_acquire::dec_ref(): internal error!");
//            #endif
//            PyThreadState_Clear(tstate);
//            PyThreadState_DeleteCurrent();
//            PYBIND11_TLS_DELETE_VALUE(detail::get_internals().tstate);
//            release = false;
//        }
//    }

//    ~gil_scoped_acquire() {
//        dec_ref();
//        if (release)
//           PyEval_SaveThread();
//    }
//private:
//    PyThreadState *tstate = nullptr;
//    bool release = true;
//};

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
    svar["Point2d"]=SvarClass::instance<Point2d>();

//    py::class_<Point3d>(m,"Point3d")
//            .def(py::init<>())
//            .def(py::init<double,double,double>())
//            .def("dot",&Point3d::dot)
//            .def("norm",&Point3d::norm)
//            .def("at",&Point3d::at)
//            .def("__repr__",&Point3d::toString)
//            .def("__str__",&Point3d::toString)
//            .def("__add__",&Point3d::add)
//            .def("__sub__",&Point3d::sub)
//            .def("__mul__",&Point3d::mul)
//            .def("__div__",&Point3d::div)
//            .def_readwrite("x", &Point3d::x)
//            .def_readwrite("y", &Point3d::y)
//            .def_readwrite("z", &Point3d::z);


//    py::class_<Point3ub>(m,"Point3ub")
//            .def(py::init<>())
//            .def(py::init<uchar,uchar,uchar>())
//            .def("dot",&Point3ub::dot)
//            .def("norm",&Point3ub::norm)
//            .def("at",&Point3ub::at)
//            .def("__repr__",&Point3ub::toString)
//            .def("__str__",&Point3ub::toString)
//            .def("__add__",&Point3ub::add)
//            .def("__sub__",&Point3ub::sub)
//            .def("__mul__",&Point3ub::mul)
//            .def("__div__",&Point3ub::div)
//            .def_readwrite("x", &Point3ub::x)
//            .def_readwrite("y", &Point3ub::y)
//            .def_readwrite("z", &Point3ub::z);


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
    svar["Camera"]=SvarClass::instance<Camera>();

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
//            .def_readwrite("x", &SO3::x)
//            .def_readwrite("y", &SO3::y)
//            .def_readwrite("z", &SO3::z)
//            .def_readwrite("w", &SO3::w)
            ;
    svar["SO3"]=SvarClass::instance<SO3>();

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
//            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
//            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
            ;
    svar["SE3"]=SvarClass::instance<SE3>();

//    py::class_<SIM3>(m,"SIM3")
//            .def(py::init<>())
//            .def(py::init<const SE3&,const double&>())
////            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
////            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
//            ;


//    py::class_<GImage>(m,"GImage",py::buffer_protocol())
//            .def(py::init<>())
//            .def(py::init<int,int,int,uchar*,bool>())
//            .def("empty",&GImage::empty)
//            .def_readonly("data",&GImage::data)
//            .def("elemSize",&GImage::elemSize)
//            .def("elemSize1",&GImage::elemSize1)
//            .def("channels",&GImage::channels)
//            .def("type",&GImage::type)
//            .def("total",&GImage::total)
//            .def("clone",&GImage::clone)
//            .def("row",&GImage::row)
//            .def("width",&GImage::getWidth)
//            .def("height",&GImage::getHeight)
//            .def("__repr__",[](const GImage& img){
//                return to_string(img.cols)+"x"
//                        +to_string(img.rows)+"x"+to_string(img.channels());})
//            .def_buffer([](GImage &m) -> py::buffer_info {
//                return py::buffer_info(
//                    m.data,                                  /* Pointer to buffer */
//                    m.elemSize1(),                           /* Size of one scalar */
//                    std::string(1,"BbHhifdd"[m.type()&0x7]), /* Python struct-style format descriptor */
//                    3,                     /* Number of dimensions */
//                    { m.rows, m.cols, m.channels()},                  /* Buffer dimensions */
//                    { m.elemSize() * m.cols,
//                      m.elemSize(), m.elemSize1() }/* Strides (in bytes) for each index */
//                );
//             })
//            .def(py::init([](py::buffer b) {
//                /* Request a buffer descriptor from Python */
//                py::buffer_info info = b.request();
//                auto idx=std::string("BbHhifd").find(info.format);
//                if(idx==std::string::npos)
//                     throw std::runtime_error("Incompatible format: expected GImage formats!");

//                if (info.ndim > 3)
//                    throw std::runtime_error("Incompatible GImage dimension!");

//                     if(info.ndim == 1)
//                          return GImage(info.shape[0],1,((idx&0x7)+((1-1)<<3)),(uchar*)info.ptr,true);
//                     if(info.ndim == 2)
//                          return GImage(info.shape[0],info.shape[1],((idx&0x7)+((1-1)<<3)),(uchar*)info.ptr,true);
//                     if(info.ndim == 3)
//                          return GImage(info.shape[0],info.shape[1],((idx&0x7)+((info.shape[2]-1)<<3)),(uchar*)info.ptr,true);

//                     return GImage();
//            }));

    Class<TicToc>("TicToc")
            .construct<>()
            .def("timestamp",&TicToc::timestamp)
            .def("tic",&TicToc::Tic)
            .def("toc",&TicToc::Toc)
            ;
    svar["TicToc"]=SvarClass::instance<TicToc>();

//    py::class_<Timer,TicToc>(m,"Timer")
//            .def(py::init<bool>())
//            .def("enter",&Timer::enter)
//            .def("leave",&Timer::leave)
//            .def_static("instance",&Timer::instance, py::return_value_policy::reference)
//            .def("disable",&Timer::disable)
//            .def("enable",&Timer::enable)
//            .def("getMeanTime",&Timer::getMeanTime)
//            .def("getStatsAsText",&Timer::getStatsAsText)
//            .def("dumpAllStats",&Timer::dumpAllStats)
//            ;

//    py::class_<Svar>(m,"Svar")
//            .def(py::init<>())
//            .def_static("instance",&Svar::instance, py::return_value_policy::reference)
//            .def_static("singleton",&Svar::instance, py::return_value_policy::reference)
//            .def("insert",&Svar::insert,py::arg("name")="",
//                 py::arg("var")="",py::arg("overwrite") = true)
////            .def("expandVal",&Svar::expandVal)
////            .def("setvar",&Svar::setvar)
//            .def("getvar",&Svar::getvar)
//            .def("parseLine",&Svar::ParseLine,"s"_a="","bSilentFailure"_a=false)
//            .def("parseStream",&Svar::ParseStream)
//            .def("parseFile",&Svar::ParseFile)
////            .def("parseMain",&Svar::ParseMain) // This have some problem
//            .def("exist",&Svar::exist)
//            .def("getInt",&Svar::GetInt, py::return_value_policy::reference,
//                 py::arg("name")="",py::arg("def") = 0)
//            .def("getDouble",&Svar::GetDouble, py::return_value_policy::reference,
//                 py::arg("name")="",py::arg("def") = 0)
//            .def("getString",&Svar::GetString, py::return_value_policy::reference,
//                 py::arg("name")="",py::arg("def") = "")
//            .def("getPointer",&Svar::GetPointer, py::return_value_policy::reference,
//                 py::arg("name")="",py::arg("def") = nullptr)
//            .def("erase",&Svar::erase)
//            .def("update",&Svar::update)
//            .def("get_data",&Svar::get_data)
//            .def("clear",&Svar::clear)
//            .def("clearAll",&Svar::clearAll)
//            .def("getStatsAsText",&Svar::getStatsAsText)
//            .def("dumpAllVars",&Svar::dumpAllVars)
//            .def("save2file",&Svar::save2file)
//            ;

    //    py::class_<MapPoint,GObject,std::shared_ptr<MapPoint> >(m,"MapPoint")
//            .def(py::init<const PointID&,const Point3d&>())
//            .def("id",&MapPoint::id)
//            .def("getPose",&MapPoint::getPose)
//            .def("setPose",&MapPoint::setPose)
//            .def("getNormal",&MapPoint::getNormal)
//            .def("setNormal",&MapPoint::setNormal)
//            .def("getColor",&MapPoint::setColor)
//            .def("getDescriptor",&MapPoint::getDescriptor)
//            .def("isPoseRelative",&MapPoint::isPoseRelative)
//            .def("refKeyframeID",&MapPoint::refKeyframeID)
//            .def("refKeyframe",&MapPoint::refKeyframe)
//            .def("observationNum",&MapPoint::observationNum)
//            .def("getObservations",(MapPointObsVec (MapPoint::*)()const) &MapPoint::getObservations)
//            .def("eraseObservation",&MapPoint::eraseObservation)
//            .def("clearObservation",&MapPoint::clearObservation)
//            .def_property("pose", &MapPoint::getPose, &MapPoint::setPose)
//            .def_property("normal", &MapPoint::getNormal, &MapPoint::setNormal)
//            .def_property("color", &MapPoint::getColor, &MapPoint::setColor)
//            .def_property("descriptor", &MapPoint::getDescriptor, &MapPoint::setDescriptor)
//            ;


//    py::class_<FrameConnection,GObject,std::shared_ptr<FrameConnection> >(m,"FrameConnection")
//            .def(py::init<>())
//            .def("matchesNum",&FrameConnection::matchesNum)
//            .def("getInformation",&FrameConnection::getInformation)
//            .def("setMatches",&FrameConnection::setMatches)
//            .def("setInformation",&FrameConnection::setInformation)
//            .def("getMatches",(std::vector<std::pair<int,int> >(FrameConnection::*)())&FrameConnection::getMatches)
//            ;

//    py::class_<MapFrame,GObject,std::shared_ptr<MapFrame>>(m,"MapFrame")
//            .def(py::init<const FrameID&,const double&>())
//            .def("id",&MapFrame::id)
//            .def("timestamp",&MapFrame::timestamp)
//            .def("setPose",(void (MapFrame::*)(const SE3&))&MapFrame::setPose)
//            .def("setPoseSim3",(void (MapFrame::*)(const SIM3&))&MapFrame::setPose)
//            .def("getPose",(SE3(MapFrame::*)()const)&MapFrame::getPose)
//            .def("getPoseScale",&MapFrame::getPoseScale)
//            .def("cameraNum",&MapFrame::cameraNum)
//            .def("getCameraPose",&MapFrame::getCameraPose)
//            .def("imageChannels",&MapFrame::imageChannels)
//            .def("getCamera",&MapFrame::getCamera,py::arg("idx") = 0)
//            .def("getImage",&MapFrame::getImage,py::arg("idx") = 0,py::arg("channalMask")=0)
//            .def("setImage",&MapFrame::setImage,py::arg("img") = GImage(),py::arg("idx") = 0,py::arg("channalMask")=0)
//            .def("setCamera",&MapFrame::setCamera,py::arg("camera")=Camera(),py::arg("idx") = 0)
//            .def("getIMUNum",&MapFrame::getIMUNum)
//            .def("getIMUPose",&MapFrame::getIMUPose)
//            .def("getAcceleration",&MapFrame::getAcceleration)
//            .def("getAngularVelocity",&MapFrame::getAngularVelocity)
//            .def("getMagnetic",&MapFrame::getMagnetic)
//            .def("getAccelerationNoise",&MapFrame::getAccelerationNoise)
//            .def("getAngularVNoise",&MapFrame::getAngularVNoise)
//            .def("getPitchYawRoll",&MapFrame::getPitchYawRoll)
//            .def("getPYRSigma",&MapFrame::getPYRSigma)
//            .def("getGPSNum",&MapFrame::getGPSNum)
//            .def("getGPSPose",&MapFrame::getGPSPose)
//            .def("getGPSLLA",&MapFrame::getGPSLLA)
//            .def("getGPSLLASigma",&MapFrame::getGPSLLASigma)
//            .def("getGPSECEF",&MapFrame::getGPSECEF)
//            .def("getHeight2Ground",&MapFrame::getHeight2Ground)
//            .def("getGPSECEF",&MapFrame::getGPSECEF)
//            .def("getHeight2Ground",&MapFrame::getHeight2Ground)
//            .def("keyPointNum",&MapFrame::keyPointNum)
//            .def("setKeyPoints",&MapFrame::setKeyPoints)
//            .def("getKeyPoints",(std::vector<KeyPoint>(MapFrame::*)() const)&MapFrame::getKeyPoints)
//            .def("getKeyPointColor",&MapFrame::getKeyPointColor)
//            .def("getKeyPointIDepthInfo",&MapFrame::getKeyPointIDepthInfo)
//            .def("getKeyPointObserve",&MapFrame::getKeyPointObserve)
//            .def("getDescriptor",&MapFrame::getDescriptor)
//            .def("getBoWVector",(BowVector (MapFrame::*)()const)&MapFrame::getBoWVector)
//            .def("getFeatureVector",(FeatureVector (MapFrame::*)()const)&MapFrame::getFeatureVector)
//            .def("getFeaturesInArea",&MapFrame::getFeaturesInArea)
//            .def("observationNum",&MapFrame::observationNum)
//            .def("getObservations",(std::map<GSLAM::PointID,size_t>(MapFrame::*)()const)&MapFrame::getObservations)
//            .def("addObservation",&MapFrame::addObservation)
//            .def("eraseObservation",&MapFrame::eraseObservation)
//            .def("clearObservations",&MapFrame::clearObservations)
//            .def("getParent",&MapFrame::getParent)
//            .def("getChild",&MapFrame::getChild)
//            .def("getParents",(FrameConnectionMap (MapFrame::*)()const)&MapFrame::getParents)
//            .def("getChildren",(FrameConnectionMap (MapFrame::*)()const)&MapFrame::getChildren)
//            .def("addParent",&MapFrame::addParent)
//            .def("addChildren",&MapFrame::addChildren)
//            .def("eraseParent",&MapFrame::eraseParent)
//            .def("eraseChild",&MapFrame::eraseChild)
//            .def("clearParents",&MapFrame::clearParents)
//            .def("clearChildren",&MapFrame::clearChildren)
//            .def("getMedianDepth",&MapFrame::getMedianDepth)
//            .def("channelTypeString",&MapFrame::channelTypeString)
//            .def("channelString",&MapFrame::channelString)
//            ;

   Class<Dataset>("Dataset")
            .construct<>()
            .construct<const std::string&>()
            .def("name",&Dataset::name)
            .def("type",&Dataset::type)
            .def("isOpened",&Dataset::isOpened)
            .def("grabFrame",&Dataset::grabFrame)
            .def("open",&Dataset::open)
            ;
   svar["Dataset"]=SvarClass::instance<Dataset>();


//    py::class_<Map,PyMap,GObject,SPtr<Map> >(m,"Map")
//            .def(py::init<>())
//            .def("insertMapPoint",&Map::insertMapPoint)
//            .def("insertMapFrame",&Map::insertMapFrame)
//            .def("eraseMapPoint",&Map::eraseMapPoint)
//            .def("eraseMapFrame",&Map::eraseMapFrame)
//            .def("clear",&Map::clear)
//            .def("frameNum",&Map::frameNum)
//            .def("pointNum",&Map::pointNum)
//            .def("getFrame",&Map::getFrame)
//            .def("getPoint",&Map::getPoint)
//            .def("getFrames",(FrameArray(Map::*)()const)&Map::getFrames)
//            .def("getPoints",(PointArray(Map::*)()const)&Map::getPoints)
//            .def("setLoopDetector",&Map::setLoopDetector)
//            .def("getLoopDetector",&Map::getLoopDetector)
//            .def("obtainCandidates",(LoopCandidates(Map::*)(const FramePtr&))&Map::obtainCandidates)
//            .def("save",&Map::save)
//            .def("load",&Map::load)
//            .def("getPid",&Map::getPid)
//            .def("getFid",&Map::getFid)
//            ;

//    py::class_<HashMap,Map,SPtr<HashMap> >(m,"HashMap")
//            .def(py::init<>())
//            ;

//    py::class_<SLAM,PySLAM,GObject,SPtr<SLAM> >(m,"SLAM")
//            .def(py::init<>())
//            .def("valid",&SLAM::valid)
//            .def("isDrawable",&SLAM::isDrawable)
//            .def("setMap",&SLAM::setMap)
//            .def("getMap",&SLAM::getMap)
//            .def("setSvar",&SLAM::setSvar)
//            .def("setCallback",&SLAM::setCallback)
//            .def("track",&SLAM::track)
//            .def("feed",&SLAM::feed)
//            .def("finalize",&SLAM::finalize)
//            .def_static("create",&SLAM::create)
//            ;

//    py::class_<Vocabulary,SPtr<Vocabulary> >(m,"Vocabulary")
//            .def(py::init<const std::string &>())
//            .def_static("create",&Vocabulary::create)
//            .def("save",&Vocabulary::save)
//            .def("load",(bool(Vocabulary::*)(const std::string &))&Vocabulary::load)
//            .def("size",&Vocabulary::size)
//            .def("empty",&Vocabulary::empty)
//            .def("clear",&Vocabulary::clear)
//            .def("transformImage",(void (Vocabulary::*)(const TinyMat&,
//                 BowVector &, FeatureVector &, int)const)&Vocabulary::transform)
//            .def("transformFeature",(void (Vocabulary::*)(const TinyMat &,
//                 WordId &, WordValue &, NodeId*, int) const)&Vocabulary::transform)
//            .def("getBranchingFactor",&Vocabulary::getBranchingFactor)
//            .def("getDepthLevels",&Vocabulary::getDepthLevels)
//            .def("getWord",&Vocabulary::getWord)
//            .def("getWordWeight",&Vocabulary::getWordWeight)
//            .def("getWeightingType",&Vocabulary::getWeightingType)
//            .def("getScoringType",&Vocabulary::getScoringType)
//            .def("setWeightingType",&Vocabulary::setWeightingType)
//            .def("getDescritorSize",&Vocabulary::getDescritorSize)
//            .def("getDescritorType",&Vocabulary::getDescritorType)
//            .def_static("meanValue",&Vocabulary::meanValue)
//            .def_static("distance",&Vocabulary::distance)
//            ;

//    py::class_<FileResource>(m,"FileResource")
//            .def_static("toHex",&FileResource::toHex)
//            .def_static("exportResourceFile",&FileResource::exportResourceFile)
//            .def_static("Register",&FileResource::Register)
//            .def_static("getResource",&FileResource::getResource)
//            .def_static("saveResource2File",&FileResource::saveResource2File)
//            ;

//    py::class_<Undistorter>(m,"Undistorter")
//            .def(py::init<Camera, Camera>())
//            .def("undistort",&Undistorter::undistort)
//            .def("undistortFast",&Undistorter::undistortFast)
//            .def("cameraIn",&Undistorter::cameraIn)
//            .def("cameraOut",&Undistorter::cameraOut)
//            .def("prepareReMap",&Undistorter::prepareReMap)
//            .def("valid",&Undistorter::valid)
//            ;

//    py::class_<TileBase, GObject, TilePtr >(m,"Tile")
//            .def(py::init<>())
//            .def("type",&TileBase::type)
//            .def("getTileImage",&TileBase::getTileImage)
//            .def("getTileHeight",&TileBase::getTileHeight)
//            .def("getTilePosition",&TileBase::getTilePosition)
//            .def("getTimeStamp",&TileBase::getTimeStamp)
//            .def("memSizeInBytes",&TileBase::memSizeInBytes)
//            .def("toStream",&TileBase::toStream)
//            .def("fromStream",&TileBase::fromStream)
//            ;

//    py::class_<TileManager, GObject, TileManagerPtr >(m,"TileManager")
//            .def("type",&TileManager::type)
//            .def("getTile",&TileManager::getTile)
//            .def("setTile",&TileManager::setTile)
//            .def("maxZoomLevel",&TileManager::maxZoomLevel)
//            .def("minZoomLevel",&TileManager::minZoomLevel)
//            .def("save",&TileManager::save)
//            ;

    Class<Messenger>("Messenger")
            .construct<>()
            .def_static("instance",&Messenger::instance)
            .def("getPublishers",&Messenger::getPublishers)
            .def("getSubscribers",&Messenger::getSubscribers)
            .def("introduction",&Messenger::introduction)
            .def("advertise",[](Messenger msg, Svar py_class,
                 const std::string& topic, int queue_size = 0){
      return msg.advertise<Svar>(topic,queue_size);
    })
    .def("subscribe",[](Messenger msger, Svar py_class,
         const std::string& topic, int queue_size,
         Svar callback){
      return msger.subscribe(topic,queue_size,[callback](Svar msg){
//         gil_scoped_acquire lock;
         if(callback.isFunction()) callback(msg);
         else if(callback.is<PyObjectHolder>()){
           PyObjectHolder holder=callback.as<PyObjectHolder>();
           SvarPy::fromPy(PyObject_Call(holder.obj, SvarPy::getPy(msg),nullptr));
         }
      });
    })
    .def("publish",&Messenger::publish<Svar>);
    svar["Messenger"]=SvarClass::instance<Messenger>();

    Class<Publisher>("Publisher")
            .def("shutdown",&Publisher::shutdown)
            .def("getTopic",&Publisher::getTopic)
            .def("getTypeName",&Publisher::getTypeName)
            .def("getNumSubscribers",&Publisher::getNumSubscribers)
            .def("publish",&Publisher::publish);
    svar["Publisher"]=SvarClass::instance<Publisher>();

    Class<Subscriber>("Subscriber")
            .def("shutdown",&Subscriber::shutdown)
            .def("getTopic",&Subscriber::getTopic)
            .def("getTypeName",&Subscriber::getTypeName)
            .def("getNumPublishers",&Subscriber::getNumPublishers);
    svar["Subscriber"]=SvarClass::instance<Subscriber>();

}


}
