#include "MapPointORB.h"


MapPointORB::MapPointORB(const PointID& id_,const Point3Type& position,
                         const ColorType& color)
    :MapPoint(id_,position),_color(color)
{

}
