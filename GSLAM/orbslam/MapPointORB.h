#ifndef MAPPOINTORB_H
#define MAPPOINTORB_H
#include "GSLAM/Map.h"

class MapPointORB:public MapPoint
{
public:
    MapPointORB(const PointID& id_,const Point3Type& position=Point3Type(0,0,0),
                const ColorType& color=ColorType(255,255,255));

    ColorType _color;
};

#endif // MAPPOINTORB_H
