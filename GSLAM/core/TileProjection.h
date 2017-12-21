#ifndef GSLAM_TILEPROJECTION_H
#define GSLAM_TILEPROJECTION_H

#include <string>
#include <GSLAM/core/GSLAM.h>

namespace GSLAM {

class Size
{
public:
    Size() { _width = 0; _height = 0; }
    Size(int w, int h) { _width = w; _height = h; }

    Size operator-(const Size &sz1){return Size(_width-sz1._width,_height-sz1._height);}
    Size operator+(const Size &sz1){return Size(sz1._width+_width,sz1._height+_height);}

    int width()const {return _width;}
    int height()const {return _height;}
    void setWidth(int const& value){_width=value;}
    void setHeight(int const& value){_height=value;}

private:
    int _width, _height;
};

class PointWithLatLng
{
public:
    PointWithLatLng()
    {
        _lat = 0;
        _lng = 0;
        _empty = true;
    }

    PointWithLatLng(const double &lat,const double &lng)
    {
        _lat = lat;
        _lng = lng;
        _empty=false;
    }


    bool isEmpty()
    {
        return _empty;
    }


    double lat()const
    {
        return _lat;
    }

    void setLat(const double &value)
    {
        _lat = value;
        _empty=false;
    }


    double lng()const
    {
        return _lng;
    }

    void setLng(const double &value)
    {
        _lng = value;
        _empty=false;
    }

private:
    double _lat;
    double _lng;
    bool _empty;
};

class GPSConverter
{
public:
    typedef PointWithLatLng Gps;
    constexpr const static double pi = 3.1415926535897932384626;
    constexpr const static double a  =  6378245.0;
    constexpr const static double ee = 0.00669342162296594323;

    /**
     * 84 to 火星坐标系 (GCJ-02) World Geodetic System ==> Mars Geodetic System
     *
     * @param lat
     * @param lon
     * @return
     */
    static Gps gps84_To_Gcj02(double lat, double lon) {
        if (outOfChina(lat, lon)) {
            return Gps(lat,lon);
        }
        double dLat = transformLat(lon - 105.0, lat - 35.0);
        double dLon = transformLon(lon - 105.0, lat - 35.0);
        double radLat = lat / 180.0 * pi;
        double magic = sin(radLat);
        magic = 1 - ee * magic * magic;
        double sqrtMagic = sqrt(magic);
        dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
        dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
        double mgLat = lat + dLat;
        double mgLon = lon + dLon;
        return Gps(mgLat, mgLon);
    }

    /**
     * * 火星坐标系 (GCJ-02) to 84 * * @param lon * @param lat * @return
     * */
    static Gps gcj_To_Gps84(double lat, double lon) {
        Gps gps = transform(lat, lon);
        double lontitude = lon * 2 - gps.lng();
        double latitude = lat * 2 - gps.lat();
        return Gps(latitude, lontitude);
    }

    /**
     * 火星坐标系 (GCJ-02) 与百度坐标系 (BD-09) 的转换算法 将 GCJ-02 坐标转换成 BD-09 坐标
     *
     * @param gg_lat
     * @param gg_lon
     */
    static Gps gcj02_To_Bd09(double gg_lat, double gg_lon) {
        double x = gg_lon, y = gg_lat;
        double z = sqrt(x * x + y * y) + 0.00002 * sin(y * pi);
        double theta = atan2(y, x) + 0.000003 * cos(x * pi);
        double bd_lon = z * cos(theta) + 0.0065;
        double bd_lat = z * sin(theta) + 0.006;
        return Gps(bd_lat, bd_lon);
    }

    /**
     * * 火星坐标系 (GCJ-02) 与百度坐标系 (BD-09) 的转换算法 * * 将 BD-09 坐标转换成GCJ-02 坐标 * * @param
     * bd_lat * @param bd_lon * @return
     */
    static Gps bd09_To_Gcj02(double bd_lat, double bd_lon) {
        double x = bd_lon - 0.0065, y = bd_lat - 0.006;
        double z = sqrt(x * x + y * y) - 0.00002 * sin(y * pi);
        double theta = atan2(y, x) - 0.000003 * cos(x * pi);
        double gg_lon = z * cos(theta);
        double gg_lat = z * sin(theta);
        return Gps(gg_lat, gg_lon);
    }

    /**
     * (BD-09)-->84
     * @param bd_lat
     * @param bd_lon
     * @return
     */
    static Gps bd09_To_Gps84(double bd_lat, double bd_lon) {

        Gps gcj02 = bd09_To_Gcj02(bd_lat, bd_lon);
        Gps map84 = gcj_To_Gps84(gcj02.lat(),gcj02.lng());
        return map84;

    }

    static Gps gps84_To_Bd09(double lat,double lon){
        Gps gcj02=gps84_To_Gcj02(lat,lon);
        return gcj02_To_Bd09(gcj02.lat(),gcj02.lng());
    }

    static bool outOfChina(double lat, double lon) {
        if (lon < 72.004 || lon > 137.8347)
            return true;
        if (lat < 0.8293 || lat > 55.8271)
            return true;
        return false;
    }

    static Gps transform(double lat, double lon) {
        if (outOfChina(lat, lon)) {
            return Gps(lat, lon);
        }
        double dLat = transformLat(lon - 105.0, lat - 35.0);
        double dLon = transformLon(lon - 105.0, lat - 35.0);
        double radLat = lat / 180.0 * pi;
        double magic = sin(radLat);
        magic = 1 - ee * magic * magic;
        double sqrtMagic = sqrt(magic);
        dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
        dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
        double mgLat = lat + dLat;
        double mgLon = lon + dLon;
        return Gps(mgLat, mgLon);
    }

    static double transformLat(double x, double y) {
        double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y
                + 0.2 * sqrt(abs(x));
        ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
        ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
        ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
        return ret;
    }

    static double transformLon(double x, double y) {
        double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1
                * sqrt(abs(x));
        ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
        ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
        ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0
                * pi)) * 2.0 / 3.0;
        return ret;
    }
};

class PureProjection : public GObject
{
public:
    virtual std::string type(){return "PureProjection";}


    /**
     * @brief axis of earth (radius of earth)
     *
     * @return radius of earth
     */
    virtual double axis()const=0;

    /**
     * @brief flattening coefficient of earth
     *
     * @return flattening coefficient
     */
    virtual double flattening()const=0;

    /**
     * @brief Get one tile's size (in pixel)
     *
     * @return usually (256,256)
     */
    virtual Size tileSize() const=0;


    /**
     * @brief Get pixel position from lat/lng
     *
     * @param lat           - latitude (y-axis)
     * @param lng           - longtitude (x-axis)
     * @param zoom          - zoom level
     *
     * @return pixel position of given lat/lng position
     */
    virtual Point2i fromLatLngToPixel(double lat, double lng, int const& zoom)=0;
    /**
     * @brief Get pixel position from lat/lng
     *
     * @param p             - lng(x) / lat(y)
     * @param zoom          - zoom level
     *
     * @return pixel position of given lat/lng position
     */
    Point2i fromLatLngToPixel(const PointWithLatLng &p,const int &zoom)
    {
        return fromLatLngToPixel(p.lat(), p.lng(), zoom);
    }


    /**
     * @brief Get tile index from given lat/lng values
     *
     * @param lat           - latitude (y-axis)
     * @param lng           - longitude (x-axis)
     * @param zoom          - zoom level
     *
     * @return tile index
     */
    virtual Point2i fromLatLngToTileXY(double lat, double lng, int const& zoom) = 0;

    Point2i fromLatLngToTileXY(const PointWithLatLng& p, const int& zoom)
    {
        return fromLatLngToTileXY(p.lat(), p.lng(), zoom);
    }

    /**
     * @brief Get lat/lng from pixel position
     *
     * @param x             - pixel position (x)
     * @param y             - pixel position (y)
     * @param zoom          - zoom level
     *
     * @return current lng(x)/lat(y) position
     */

    virtual PointWithLatLng fromPixelToLatLng(const int &x,const int &y,const int &zoom)=0;
    /**
     * @brief Get lat/lng from pixel position
     *
     * @param p             - pixel position
     * @param zoom          - zoom level
     *
     * @return current lng(x)/lat(y) position
     */
    PointWithLatLng fromPixelToLatLng(const pi::Point2i &p,const int &zoom)
    {
        return fromPixelToLatLng(p.x, p.y, zoom);
    }


    /**
     * @brief Get tile index from pixel position
     *
     * @param p             - pixel position
     *
     * @return current active tile position
     */
    virtual Point2i fromPixelToTileXY(const pi::Point2i &p)
    {
        return pi::Point2i((int) (p.x / tileSize().width()), (int) (p.y / tileSize().height()));
    }

    /**
     * @brief Get pixel position from current tile index
     *
     * @param p             - tile index
     *
     * @return current tile's pixel position
     */
    virtual Point2i fromTileXYToPixel(const pi::Point2i &p)
    {
        return pi::Point2i((p.x * tileSize().width()), (p.y * tileSize().height()));
    }


    virtual  Size getTileMatrixMinXY(const int &zoom)=0;
    virtual  Size getTileMatrixMaxXY(const int &zoom)=0;

    virtual Size getTileMatrixSizeXY(const int &zoom);
    int getTileMatrixItemCount(const int &zoom);

    virtual Size getTileMatrixSizePixel(const int &zoom);


    virtual double getGroundResolution(const int &zoom, const double &latitude);
    virtual int getBestZoom(const double &gsd, const double &lat);

    double degreesToRadians(const double &deg)const
    {
        return (D2R * deg);
    }

    double radiansToDegrees(const double &rad)const
    {
        return (R2D * rad);
    }

    void fromGeodeticToCartesian(double Lat,double Lng,double Height,  double &X,  double &Y,  double &Z);
    void fromCartesianTGeodetic(const double &X,const double &Y,const double &Z,  double &Lat,  double &Lng);
    static double distanceBetweenLatLng(PointWithLatLng const& p1, PointWithLatLng const& p2);

    /**
     * @brief Get maximum zoom level
     *
     * @return maximum zoom level (default is 23)
     */
    virtual int zoomMax(void) const;


protected:
    constexpr static const double PI= M_PI;
    constexpr static const double HALF_PI= (M_PI * 0.5);
    constexpr static const double TWO_PI= (M_PI * 2.0);
    constexpr static const double EPSLoN= 1.0e-10;
    constexpr static const double MAX_VAL= 4;
    constexpr static const double maxLong= 2147483647;
    constexpr static const double DBLLONG= 4.61168601e18;
    constexpr static const double R2D=180.0/M_PI;
    constexpr static const double D2R=M_PI/180.0;
    constexpr static const int    ZOOMMAX= 22;

    static double sign(const double &x);

    static double adjustLongitude(double x);
    static void SinCos(const double &val,  double &sin, double &cos);
    static double e0fn(const double &x);
    static double e1fn(const double &x);
    static double e2fn(const double &x);
    static double e3fn(const double &x);
    static double mlfn(const double &e0,const double &e1,const double &e2,const double &e3,const double &phi);
    static int64_t getUTMzone(const double &lon);
};
typedef SPtr<PureProjection> TileProjectionPtr;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class MercatorProjection : public PureProjection
{
public:
    MercatorProjection();
    virtual std::string type(){return "MercatorProjection";}

    virtual Size tileSize() const;
    virtual double axis() const;
    virtual double flattening()const;

    virtual pi::Point2i fromLatLngToPixel(double lat, double lng, int const& zoom);
    virtual pi::Point2i fromLatLngToTileXY(double lat, double lng, int const& zoom);
    virtual PointWithLatLng fromPixelToLatLng(const int &x,const int &y,const int &zoom);
    virtual Size getTileMatrixMinXY(const int &zoom);
    virtual Size getTileMatrixMaxXY(const int &zoom);

private:
    double clip(double const& n, double const& minValue, double const& maxValue)const;

    const double MinLatitude;
    const double MaxLatitude;
    const double MinLongitude;
    const double MaxLongitude;

    Size _tileSize;
};
typedef SPtr<MercatorProjection> MecatorProjectionPtr;

class GCJ02Projection: public MercatorProjection
{
public:
    virtual pi::Point2i fromLatLngToPixel(double lat, double lng, int const& zoom)
    {
        auto gcj=GPSConverter::gps84_To_Gcj02(lat,lng);
        return MercatorProjection::fromLatLngToPixel(gcj.lat(),gcj.lng(),zoom);
    }

    virtual PointWithLatLng fromPixelToLatLng(const int &x,const int &y,const int &zoom)
    {
        PointWithLatLng gcj=MercatorProjection::fromPixelToLatLng(x,y,zoom);
        return GPSConverter::gcj_To_Gps84(gcj.lat(),gcj.lng());
    }
};
typedef SPtr<GCJ02Projection> GCJ02ProjectionPtr;

class BaiduProjection : public MercatorProjection
{
    virtual pi::Point2i fromLatLngToPixel(double lat, double lng, int const& zoom)
    {
        auto bd=GPSConverter::gps84_To_Bd09(lat,lng);
        return MercatorProjection::fromLatLngToPixel(bd.lat(),bd.lng(),zoom);
    }

    virtual PointWithLatLng fromPixelToLatLng(const int &x,const int &y,const int &zoom)
    {
        PointWithLatLng bd=MercatorProjection::fromPixelToLatLng(x,y,zoom);
        return GPSConverter::bd09_To_Gps84(bd.lat(),bd.lng());
    }
};
typedef SPtr<BaiduProjection> BaiduProjectionPtr;

inline Size PureProjection::getTileMatrixSizeXY(const int &zoom)
{
    Size sMin = getTileMatrixMinXY(zoom);
    Size sMax = getTileMatrixMaxXY(zoom);

    return  Size(sMax.width() - sMin.width() + 1, sMax.height() - sMin.height() + 1);
}

inline int PureProjection::getTileMatrixItemCount(const int &zoom)
{
    Size s = getTileMatrixSizeXY(zoom);
    return (s.width() * s.height());
}

inline Size PureProjection::getTileMatrixSizePixel(const int &zoom)
{
    Size s = getTileMatrixSizeXY(zoom);
    return Size(s.width() * tileSize().width(), s.height() * tileSize().height());
}




inline double PureProjection::getGroundResolution(const int &zoom, const double &latitude)
{
    return (cos(latitude * (PI / 180)) * 2 * PI * axis()) / (1.0*getTileMatrixSizePixel(zoom).width());
}

inline int PureProjection::getBestZoom(const double &gsd, const double &lat)
{
    int n = (int)( (cos(lat * (PI / 180)) * 2 * PI * axis()) / (gsd*tileSize().width()) - 1);
    int z;

    if( n <= 0 )
        z = 0;
    else
        z = log(n) / log(2);

    if( z > zoomMax() ) z = zoomMax();

    return z;
}


inline double PureProjection::sign(const double &x)
{
    if(x < 0.0)
        return (-1);
    else
        return (1);
}

inline double PureProjection::adjustLongitude(double x)
{
    int64_t count = 0;

    while(true)
    {
        if(fabs(x) <= PI)
            break;
        else
            if(((int64_t) fabs(x / PI)) < 2)
                x = x - (sign(x) * TWO_PI);

            else
                if(((int64_t) fabs(x / TWO_PI)) < maxLong)
                {
                    x = x - (((int64_t) (x / TWO_PI)) * TWO_PI);
                }
                else
                    if(((int64_t) fabs(x / (maxLong * TWO_PI))) < maxLong)
                    {
                        x = x - (((int64_t) (x / (maxLong * TWO_PI))) * (TWO_PI * maxLong));
                    }
                    else
                        if(((int64_t) fabs(x / (DBLLONG * TWO_PI))) < maxLong)
                        {
                            x = x - (((int64_t) (x / (DBLLONG * TWO_PI))) * (TWO_PI * DBLLONG));
                        }
                        else
                            x = x - (sign(x) * TWO_PI);
        count++;
        if(count > MAX_VAL)
            break;
    }

    return (x);
}

inline void PureProjection::SinCos(const double &val,  double &si, double &co)
{
    si = sin(val);
    co = cos(val);
}

inline double PureProjection::e0fn(const double &x)
{
    return (1.0 - 0.25 * x * (1.0 + x / 16.0 * (3.0 + 1.25 * x)));
}

inline double PureProjection::e1fn(const double &x)
{
    return (0.375 * x * (1.0 + 0.25 * x * (1.0 + 0.46875 * x)));
}

inline double PureProjection::e2fn(const double &x)
{
    return (0.05859375 * x * x * (1.0 + 0.75 * x));
}

inline double PureProjection::e3fn(const double &x)
{
    return (x * x * x * (35.0 / 3072.0));
}

inline double PureProjection::mlfn(const double &e0,const double &e1,const double &e2,const double &e3,
                            const double &phi)
{
    return (e0 * phi - e1 * sin(2.0 * phi) + e2 * sin(4.0 * phi) - e3 * sin(6.0 * phi));
}

inline int64_t PureProjection::getUTMzone(const double &lon)
{
    return ((int64_t) (((lon + 180.0) / 6.0) + 1.0));
}


inline void PureProjection::fromGeodeticToCartesian(double Lat,double Lng,double Height,
                                             double &X,  double &Y,  double &Z)
{
    Lat = (PI / 180) * Lat;
    Lng = (PI / 180) * Lng;

    double B = axis() * (1.0 - flattening());
    double ee = 1.0 - (B / axis()) * (B / axis());
    double N = (axis() / sqrt(1.0 - ee * sin(Lat) * sin(Lat)));

    X = (N + Height) * cos(Lat) * cos(Lng);
    Y = (N + Height) * cos(Lat) * sin(Lng);
    Z = (N * (B / axis()) * (B / axis()) + Height) * sin(Lat);
}

inline void PureProjection::fromCartesianTGeodetic(const double &X,const double &Y,const double &Z,
                                            double &Lat,  double &Lng)
{
    double E = flattening() * (2.0 - flattening());
    Lng = atan2(Y, X);

    double P = sqrt(X * X + Y * Y);
    double Theta = atan2(Z, (P * (1.0 - flattening())));
    double st = sin(Theta);
    double ct = cos(Theta);
    Lat = atan2(Z + E / (1.0 - flattening()) * axis() * st * st * st,
                P - E * axis() * ct * ct * ct);

    Lat /= (PI / 180);
    Lng /= (PI / 180);
}

inline double PureProjection::distanceBetweenLatLng(PointWithLatLng const& p1, PointWithLatLng const& p2)
{
    double R = 6371; // km
    double lat1=p1.lat();
    double lat2=p2.lat();
    double lon1=p1.lng();
    double lon2=p2.lng();
    double dLat = (lat2-lat1)* (PI / 180);
    double dLon = (lon2-lon1)* (PI / 180);
    double a = sin(dLat/2) * sin(dLat/2) +
            cos(lat1* (PI / 180)) * cos(lat2* (PI / 180)) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = R * c;
    return d;
}


inline int PureProjection::zoomMax(void) const
{
    return ZOOMMAX;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline MercatorProjection::MercatorProjection() :
    MinLatitude(-85.05112878), MaxLatitude(85.05112878),
    MinLongitude(-177), MaxLongitude(177),
    _tileSize(256, 256)
{
}

inline pi::Point2i MercatorProjection::fromLatLngToPixel(double lat, double lng, const int &zoom)
{
    pi::Point2i ret;// = Point.Empty;

    lat = clip(lat, MinLatitude, MaxLatitude);
    lng = clip(lng, MinLongitude, MaxLongitude);

    double x = (lng + 180) / 360;
    double sinLatitude = sin(lat * M_PI / 180);
    double y = 0.5 - log((1 + sinLatitude) / (1 - sinLatitude)) / (4 * M_PI);

    Size s = getTileMatrixSizePixel(zoom);
    int mapSizeX = s.width();
    int mapSizeY = s.height();

    ret.x = (int) clip(x * mapSizeX + 0.5, 0, mapSizeX - 1);
    ret.y = (int) clip(y * mapSizeY + 0.5, 0, mapSizeY - 1);

    return ret;
}

inline pi::Point2i MercatorProjection::fromLatLngToTileXY(double lat, double lng, int const& zoom)
{
    pi::Point2i ps = fromLatLngToPixel(lat, lng, zoom);
    return pi::Point2i(ps.x/_tileSize.width(), ps.y/_tileSize.height());
}


inline PointWithLatLng MercatorProjection::fromPixelToLatLng(const int &x, const int &y, const int &zoom)
{
    PointWithLatLng ret;// = internals::PointLatLng.Empty;

    Size s = getTileMatrixSizePixel(zoom);
    double mapSizeX = s.width();
    double mapSizeY = s.height();

    double xx = (clip(x, 0, mapSizeX - 1) / mapSizeX) - 0.5;
    double yy = 0.5 - (clip(y, 0, mapSizeY - 1) / mapSizeY);

    ret.setLat(90 - 360 * atan(exp(-yy * 2 * M_PI)) / M_PI);
    ret.setLng(360 * xx);

    return ret;
}

template<class T>
T min(const T& v1, const T& v2)
{
    if( v1 < v2 ) return v1;
    else return v2;
}

template<class T>
T max(const T& v1, const T& v2)
{
    if( v1 < v2 ) return v2;
    else return v1;
}

inline double MercatorProjection::clip(const double &n, const double &minValue, const double &maxValue) const
{
    return min(max(n, minValue), maxValue);
}

inline Size MercatorProjection::tileSize() const
{
    return _tileSize;
}

inline double MercatorProjection::axis() const
{
    return 6378137;
}

inline double MercatorProjection::flattening() const
{
    return (1.0 / 298.257223563);
}

inline Size MercatorProjection::getTileMatrixMaxXY(const int &zoom)
{
    int xy = (1 << zoom);
    return  Size(xy - 1, xy - 1);
}

inline Size MercatorProjection::getTileMatrixMinXY(const int &zoom)
{
    return Size(0, 0);
}

}

#endif
