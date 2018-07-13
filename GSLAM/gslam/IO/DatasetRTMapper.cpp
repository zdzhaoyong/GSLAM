#include "GSLAM/core/Dataset.h"
#include "GSLAM/core/VideoFrame.h"
#include "GSLAM/core/Svar.h"
#include "GSLAM/core/VecParament.h"
#include "GSLAM/core/Timer.h"
#include "GSLAM/core/XML.h"

#include <list>

#ifdef HAS_QT
#include <QFileInfo>
#include <QImage>
#include <QDir>

using namespace std;
using namespace GSLAM;

inline GSLAM::Camera camFromName(string name,Svar& var)
{
    VecParament<double> paras;
    paras=var.get_var(name+".Paraments",paras);
    return GSLAM::Camera(paras.data);
}


enum RTMapperFrameStatus
{
    SLAMFRAME_READY,
    SLAMFRAME_PROCESSING,
    SLAMFRAME_PROCESSED_SUCCESS,
    SLAMFRAME_PROCESSED_FAILED
};

class RTMapperFrame : public GSLAM::MapFrame
{
public:
    RTMapperFrame(const GSLAM::FrameID& id=0,const double& timestamp=0)
        :GSLAM::MapFrame(id,timestamp),_status(SLAMFRAME_READY){}

    virtual std::string type()const{return "RTMapperFrame";}

    virtual int     cameraNum()const{return 1;}                                      // Camera number
    virtual GSLAM::SE3     getCameraPose(int idx=0) const{return GSLAM::SE3();}                    // The transform from camera to local
    virtual int     imageChannels(int idx=0) const{return GSLAM::IMAGE_BGRA;}               // Default is a colorful camera
    virtual GSLAM::Camera  getCamera(int idx=0){return _camera;}                            // The camera model
    virtual GSLAM::GImage  getImage(int idx,int channalMask){
        if(idx==0)
        {
            if(_image.empty())
            {
                using namespace GSLAM;
                QImage qimage(_imagePath.c_str());
                if(qimage.format()==QImage::Format_RGB32)
                {
                    return GImage(qimage.height(),qimage.width(),
                               GImageType<uchar,4>::Type,qimage.bits(),true);
                }
                else if(qimage.format()==QImage::Format_RGB888){
                    return GImage(qimage.height(),qimage.width(),
                               GImageType<uchar,3>::Type,qimage.bits(),true);
                }
                else if(qimage.format()==QImage::Format_Indexed8)
                {
                    return GImage(qimage.height(),qimage.width(),
                               GImageType<uchar,1>::Type,qimage.bits(),true);
                }
            }
            else  
				return _image;
        }
        else return _thumbnail;
    }   // Just return the image if only one channel is available

    // When the frame contains IMUs or GPSs
    virtual int     getIMUNum()const{return (_gpshpyr.size()==11||_gpshpyr.size()==12||_gpshpyr.size()==14)?1:0;}
    virtual bool    getAcceleration(GSLAM::Point3d& acc,int idx=0)const{return false;}        // m/s^2
    virtual bool    getAngularVelocity(GSLAM::Point3d& angularV,int idx=0)const{return false;}// rad/s
    virtual bool    getMagnetic(GSLAM::Point3d& mag,int idx=0)const{return false;}            // gauss
    virtual bool    getAccelerationNoise(GSLAM::Point3d& accN,int idx=0)const{return false;}
    virtual bool    getAngularVNoise(GSLAM::Point3d& angularVN,int idx=0)const{return false;}
    virtual bool    getPitchYawRoll(GSLAM::Point3d& pyr,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyr=GSLAM::Point3d(_gpshpyr[5],_gpshpyr[6],_gpshpyr[7]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]) {pyr=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyr=GSLAM::Point3d(_gpshpyr[6],_gpshpyr[7],_gpshpyr[8]);return true;}
        return false;
    }     // in rad
    virtual bool    getPYRSigma(GSLAM::Point3d& pyrSigma,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyrSigma=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]) {pyrSigma=GSLAM::Point3d(_gpshpyr[11],_gpshpyr[12],_gpshpyr[13]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyrSigma=GSLAM::Point3d(_gpshpyr[9],_gpshpyr[10],_gpshpyr[11]);return true;}
        return false;
    }    // in rad

    virtual int     getGPSNum()const{return (_gpshpyr.size()>=6&&_gpshpyr[3]<10)?1:0;}
    virtual bool    getGPSLLA(GSLAM::Point3d& LonLatAlt,int idx=0)const{
        if(getGPSNum()==0) return false;
        LonLatAlt=GSLAM::Point3d(_gpshpyr[0],_gpshpyr[1],_gpshpyr[2]);
        return _gpshpyr[3]<10;
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

    virtual void call(const std::string &command, void *arg)
    {
        if("GetImagePath"==command)
        {
            if(arg) *(std::string*)arg=_imagePath;
        }
        else if("ReleaseImage"==command)
        {
            _image=GSLAM::GImage();
        }
        else if("GetGPS"==command)
        {
            if((!arg)||_gpshpyr.empty()) return;
            std::vector<double>* dest=(std::vector<double>*)arg;
            *dest=_gpshpyr;
        }
    }

    virtual RTMapperFrame& operator =(const RTMapperFrame& frame)
    {
        this->_camera = frame._camera;
        this->_cameraName = frame._cameraName;
        this->_gpshpyr = frame._gpshpyr;
        this->_imagePath = frame._imagePath;

        return *this;
    }

    std::string imagePath(){return _imagePath;}
    GSLAM::GImage& thumbnail(){return _thumbnail;}

    std::string         _imagePath;       // where image come from or where to cache
    std::string         _cameraName;
    GSLAM::GImage       _image,_thumbnail;// should be RGBA
    GSLAM::Camera       _camera;
    GSLAM::FramePtr     _slamFrame;
    RTMapperFrameStatus _status;
    std::vector<double> _gpshpyr;
    // 6 : long,lat,alt,sigmaX,sigmaY,sigmaZ
    // 8 : long,lat,alt,sigmaX,sigmaY,sigmaZ,Height,sigmaH
    // 11: long,lat,alt,sigmaH,sigmaV,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
    // 12: long,lat,alt,sigmaX,sigmaY,sigmaZ,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
    // 14: long,lat,alt,sigmaX,sigmaY,sigmaZ,Height,sigmaH,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
};

class DatasetRTMapper : public Dataset
{
public:
    DatasetRTMapper(const std::string& name="")
        :_frameId(0),
         _name(name)
    {
    }

    ~DatasetRTMapper(){
        _shouldStop=true;
        while(!_prepareThread.joinable()) GSLAM::Rate::sleep(0.01);
        _prepareThread.join();
    }

    std::string type() const {return "DatasetRTMapper";}

    virtual bool open(const string &dataset)
    {
        Svar var;
        bool ret = false;

        // try XML format
        ret = openRTM_XML(var, dataset);
        if( !ret )
        {
            // try SVar format
            ret = var.ParseFile(dataset);
            if( !ret ) return false;

            _seqTop = Svar::getFolderPath(dataset);
            if( var.exist("VideoReader.VideoFile") )
                ret = openRTM_Svar(var,"VideoReader");
            else
                ret = openRTM_Svar(var,"Dataset");
        }

        // set local variables & reading thread
        _frameId = 0;
        _shouldStop = false;
        _prepareThread = std::thread(&DatasetRTMapper::run, this);

        return ret;
    }


    bool isOpened()
    {
        return _camera.isValid() && _frames.size();
    }

    bool openRTM_Svar(Svar& var, const std::string& name)
	{
        _frames.clear();
        _name = name;
        string prjFile = var.GetString("Svar.ParsingFile", "");

        // load camera info
        string cameraName;
        if(var.exist("VideoReader.Camera"))
        {
            cameraName=var.GetString("VideoReader.Camera","");
        }
        else if(var.exist("Dataset.Camera"))
            cameraName=var.GetString("Dataset.Camera","");
        if(cameraName.empty()) return false;

        VecParament<double> camParas=var.get_var(cameraName+".Paraments",VecParament<double>());
        GSLAM::Camera camera(camParas.data);
        if(!camera.isValid()) return false;
        _cameraName = cameraName;
        _camera = camera;


        // load image list
        QFileInfo rtmFileInfo(QString::fromLocal8Bit(prjFile.c_str()));
        QDir      rtmFileDir=rtmFileInfo.absoluteDir();
        QFileInfo imageListInfo(rtmFileDir.absolutePath()+"/imageLists.txt");
        if(!imageListInfo.exists()) return false;

        // FIXME: image list use UTF-8 ?
        QByteArray baImgList = imageListInfo.absoluteFilePath().toLocal8Bit();
        ifstream ifs(baImgList.data());
        if(!ifs.is_open()) {
            LOG(ERROR) << "Can not open image list: " << baImgList.data();
            return false;
        }

        string line;
        while( getline(ifs, line) )
        {
            int dotIdx=line.find(',');
            if(dotIdx==string::npos) continue;

            string imgFile=line.substr(0,dotIdx);
            string paras  =line.substr(dotIdx+1);
            VecParament<double> gpsInfo;
            gpsInfo.fromString(paras);
            if(gpsInfo.size()<1) continue;

            SPtr<RTMapperFrame> frame(new RTMapperFrame(_frames.size(),gpsInfo[0]));
            frame->_cameraName=cameraName;
            frame->_camera=camera;
            gpsInfo.data.erase(gpsInfo.data.begin());
            frame->_gpshpyr=gpsInfo.data;

            QFileInfo info(QString::fromUtf8(imgFile.c_str()));
            if(info.isAbsolute()&&info.exists())
            {
                // FIXME: convert utf-8 string to local string
                QString fnUnicode = QString::fromUtf8(imgFile.c_str());
                QByteArray baFilename = fnUnicode.toLocal8Bit();
                frame->_imagePath = baFilename.data();
            }
            else
            {
                QString fnUnicode = QString::fromUtf8(imgFile.c_str());

                QByteArray baImgPath = QFileInfo(rtmFileDir.absolutePath()+"/"+fnUnicode).absoluteFilePath().toLocal8Bit();
                frame->_imagePath = baImgPath.data();
            }

            _frames.push_back(frame);
        }

        return _frames.size();
    }


    bool exportEle(GSLAM::Svar& var,tinyxml2::XMLElement* ele,string parentName="")
    {
        if( !ele ) return false;
        if( ele->Attribute("value") )
            var.insert((parentName.empty()?string():parentName+".")+ele->Name(),ele->Attribute("value"));

        tinyxml2::XMLElement* child=ele->FirstChildElement();
        while(child)
        {
            exportEle(var,child,(parentName.empty()?string():parentName+".")+ele->Name());
            child=child->NextSiblingElement();
        }

        return true;
    }

    bool exportFrame(GSLAM::Svar& var,tinyxml2::XMLElement* ele,const std::string& dataset,string parentName="")
    {
        if(!ele)  return false;

        string projectType=var.GetString("ProjectType","");

        // load camera settings
        string cameraName = var.GetString("Dataset.Camera","");
        if( cameraName.empty() ) return false;

        VecParament<double> camParas=var.get_var(cameraName+".Paraments",VecParament<double>());
        GSLAM::Camera camera(camParas.data);
        if( !camera.isValid() ) return false;
        _cameraName = cameraName;
        _camera = camera;

        // load image list
        QFileInfo rtmFileInfo(QString::fromLocal8Bit(dataset.c_str()));
        QDir      rtmFileDir = rtmFileInfo.absoluteDir();

        tinyxml2::XMLElement* child=ele->FirstChildElement();
        while(child)
        {
            double timestamp=child->DoubleAttribute("timestamp");
            SPtr<RTMapperFrame> frame(new RTMapperFrame(_frames.size(),timestamp));

            // FIXME: Get image path (convert from UTF-8 to local)
            string imgFile = child->Attribute("image");
            QString fnUni = QString::fromUtf8(imgFile.c_str());
            QByteArray baImgFilename = fnUni.toLocal8Bit();
            imgFile = baImgFilename.data();

            QFileInfo info(fnUni);
            if(info.isAbsolute()&&info.exists())
            {
                frame->_imagePath = imgFile;
            }
            else
            {
                QByteArray baImgPath = QFileInfo(rtmFileDir.absolutePath()+"/"+ fnUni).absoluteFilePath().toLocal8Bit();
                frame->_imagePath = baImgPath.data();
            }
            frame->_cameraName=cameraName;
            frame->_camera=camera;

            vector<string> paras[5]={{"gps","longtitude","latitude","altitude"},
                                     {"gpsSigma","longtitude","latitude","altitude"},
                                     {"height","value","sigma"},
                                     {"attitude","pitch","yaw","roll"},
                                     {"attitudeSigma","pitch","yaw","roll"}};

            for(int i=0;i<5;i++)
            {
                tinyxml2::XMLElement* child1=child->FirstChildElement(paras[i][0].c_str());
                if(!child1) continue;
                for(int j=1;j<paras[i].size();j++)
                {
                    double gpsData = child1->DoubleAttribute(paras[i][j].c_str());
                    frame->_gpshpyr.push_back(gpsData);
                }
            }


            _frames.push_back(frame);

            child = child->NextSiblingElement();
        }

        return _frames.size();
    }


    bool openRTM_XML(GSLAM::Svar& var,const std::string& dataset)
    {
        _name = "Dataset";

        tinyxml2::XMLDocument doc;
        if(tinyxml2::XML_SUCCESS!=doc.LoadFile(dataset.c_str())) return false;

        tinyxml2::XMLElement* proiEle=doc.FirstChildElement("project");
        proiEle->SetName("");
        if(exportEle(var,proiEle))
        {
            tinyxml2::XMLElement* imgsEle = proiEle->NextSiblingElement("images");
            imgsEle->SetName("");
            return exportFrame(var,imgsEle,dataset);
        }

        return false;
    }

    GSLAM::GImage imread(const QString& imgFile)
    {
        GSLAM::GImage img;
        QImage qimage(imgFile);

        if( qimage.format() == QImage::Format_RGB32 )
        {
            img=GImage(qimage.height(), qimage.width(),
                       GImageType<uchar,4>::Type, qimage.bits(), true);
        }
        else if(qimage.format() == QImage::Format_RGB888){
            img=GImage(qimage.height(), qimage.width(),
                       GImageType<uchar,3>::Type, qimage.bits(), true);
        }
        else if(qimage.format()==QImage::Format_Indexed8)
        {
            img=GImage(qimage.height(), qimage.width(),
                       GImageType<uchar,1>::Type, qimage.bits(), true);
        }

        return img;
    }

    void run()
    {
        while (!_shouldStop)
        {
            if(_preparedFrames.size()>=2)
            {
                Rate::sleep(0.001);
                continue;
            }

            auto fr = prepareFrame();
            if( !fr ) break;

            _preparedFrames.push_back(fr);
            _eventPrepared.set();
        }

        _shouldStop = true;
    }

    GSLAM::FramePtr grabFrame()
    {
        if(_preparedFrames.size())
        {
            auto ret = _preparedFrames.front();
            _preparedFrames.pop_front();
            return ret;
        }

        if( _shouldStop ) return GSLAM::FramePtr();

        _eventPrepared.wait();
        auto ret = _preparedFrames.front();
        _preparedFrames.pop_front();

        return ret;
    }

    GSLAM::FramePtr prepareFrame()
    {
        if( _frameId < _frames.size() )
        {
            SPtr<RTMapperFrame> frame = _frames[_frameId++];

            SPtr<RTMapperFrame> nf(new RTMapperFrame(frame->id(), frame->timestamp()));
            *nf = *frame;
            nf->_image = imread(nf->_imagePath.c_str());

            return nf;
        }

        return GSLAM::FramePtr(NULL);
    }

    string           _seqTop;
    GSLAM::FrameID   _frameId;
    GSLAM::Camera    _camera;
    string           _name, _cameraName;

    std::vector<SPtr<RTMapperFrame> >   _frames;

    std::thread      _prepareThread;
    bool             _shouldStop;
    GSLAM::Event     _eventPrepared;
    std::list<GSLAM::FramePtr> _preparedFrames;
};

REGISTER_DATASET(DatasetRTMapper,rtm)


#endif // end of HAS_QT
