#include <vector>
#include <iomanip>
#include <iostream>

#include "Point.h"

namespace GSLAM{

typedef pi::Point3d Point3d;

// WGS84 Ellipsoid
static const double WGS84_A = 6378137.0;      // major axis
static const double WGS84_B = 6356752.314245; // minor axis
static const double WGS84_E = 0.0818191908;   // first eccentricity

template<typename T> inline T Square( T x ){return x * x;}
inline double D2R( double degree ){return degree * M_PI / 180.0;}
inline double R2D( double radian ){return radian / M_PI * 180.0;}

template <typename T=Point3d>
class GPS
{
public:
    GPS(const std::string& nameGPS="GPS"):name(nameGPS){}
    virtual ~GPS(){}

    virtual bool    insert(double time,const T& gpsData){return false;}
    virtual size_t  size()const{return 0;}
    virtual bool    getArray(std::vector<T>& gpsArray){return false;} /// return all data
    virtual T       at(size_t idx){return T();}
    virtual T       atTime(const double& time=-1,bool nearist=true){return T();}
    virtual void    getTimeRange(double& minTime, double& maxTime){minTime=-1;maxTime=-1;}

    virtual bool    load(const std::string& filename){return false;}
    virtual bool    save(const std::string& filename){return false;}

    inline bool  hasTime(const double& time){
        double minTime,maxTime;
        getTimeRange(minTime,maxTime);
        return time>=minTime&&time<=maxTime;
    }

    /**
     ** Convert WGS84 lon,lat,alt data to ECEF data (Earth Centered Earth Fixed)
     ** @param lat Latitude in degree
     ** @param lon Longitude in degree
     ** @param alt Altitude relative to the WGS84 ellipsoid
     ** @return ECEF corresponding coordinates
     **
     ** http://fr.mathworks.com/matlabcentral/newsreader/view_thread/142629
     **/
    static pi::Point3d GPS2XYZ(double lat,double lon,double alt)
    {
      const double clat = cos( D2R(lat) );
      const double slat = sin( D2R(lat) );
      const double clon = cos( D2R(lon) );
      const double slon = sin( D2R(lon) );

      const double a2 = Square(WGS84_A);
      const double b2 = Square(WGS84_B);

      const double L = 1.0 / sqrt(a2 * Square(clat) + b2 * Square(slat));
      const double x = (a2 * L + alt) * clat * clon;
      const double y = (a2 * L + alt) * clat * slon;
      const double z = (b2 * L + alt) * slat;

      return pi::Point3d(x, y, z);
    }

    static pi::Point3d XYZ2GPS(double x,double y,double z)
    {
      const double b = sqrt(WGS84_A*WGS84_A*(1-WGS84_E*WGS84_E));
      const double ep = sqrt((WGS84_A*WGS84_A-b*b)/(b*b));
      const double p = sqrt(x*x+y*y);
      const double th = atan2(WGS84_A*z,b*p);
      const double lon = atan2(y,x);
      const double lat = atan2((z+ep*ep*b* pow(sin(th),3)),(p-WGS84_E*WGS84_E*WGS84_A*pow(cos(th),3)));
      const double N = WGS84_A/sqrt(1-WGS84_E*WGS84_E*sin(lat)*sin(lat));
      const double alt = p/cos(lat)-N;

      return pi::Point3d(R2D(lat), R2D(lon), alt);
    }

    static pi::Point3d GPS2XYZ(pi::Point3d gps){return GPS2XYZ(gps.x,gps.y,gps.z);}
    static pi::Point3d XYZ2GPS(pi::Point3d xyz){return XYZ2GPS(xyz.x,xyz.y,xyz.z);}

    std::string  name;
};

template <typename T>
class GPSArray:public GPS<T>
{
public:
    GPSArray(const std::string& nameGPS="GPS"):GPS<T>(nameGPS){}

    virtual bool insert(double time,const T& gpsData){
        if(time>data.back().first)
        {
            data.push_back(gpsData);
            return true;
        }
        return false;
    }

    virtual size_t  size()const{return data.size();}
    virtual bool    getArray(std::vector<T>& gpsArray){
        gpsArray.clear();
        gpsArray.reserve(data.size());
        for(std::pair<double,T>& d:data) gpsArray.push_back(d.second);
        return true;
    } /// return all data

    virtual T       at(size_t idx){return data[idx].second;}
    virtual T       atTime(const double& time=-1,bool nearist=true);
    virtual void    getTimeRange(double& minTime, double& maxTime){
        if(!data.size())
        {
            minTime=-1;maxTime=-1;
        }
        else{
            minTime=data.front().first;
            maxTime=data.back().first;
        }
    }

private:
    std::vector<std::pair<double,T> > data;
};

template <typename T>
T GPSArray<T>::atTime(const double& time,bool nearist)
{
    if(!data.size()) return false;

    if(!GPS<T>::hasTime(time)) return T();

    int idxMin=0,idxMax=data.size()-2;


    while(idxMax-idxMin>16)// fast up with idx approciate
    {
        int idx=idxMin+(idxMax-idxMin)*(time-data[idxMin].first)/(data[idxMax].first-data[idxMin].first);
        if(data[idx].first>=time) idxMax=idx;
        else idxMin=idx;
    }

    while(idxMax-idxMin>1)
    {
        int idx=(idxMax+idxMin)>>1;
        if(data[idx].first>time) idxMax=idx;
        else idxMin=idx;
    }


    if(nearist)
    {
        double timeDiffLow =time-data[idxMin].first;
        double timeDiffHigh=data[idxMax].first-time;
        if(timeDiffLow<timeDiffHigh)
        {
            if(timeDiffLow>1.0) return T();
            //less than one second
            return data[idxMin].second;
        }
        else
        {
            if(timeDiffHigh>1.0) return T();
            return data[idxMax].second;
        }
    }
    else
    {
        std::pair<double,T>& low=data[idxMin];
        std::pair<double,T>& up =data[idxMax];
        double timeDiffAll=up.first-low.first;
        if(timeDiffAll>1.0) return false;

        double kLow=((double)(time-low.first))/timeDiffAll;
        double kUp =1.-kLow;
        return (low.second*kLow+up.second*kUp);
    }
    return T();
}

}
