#ifndef SE3_GSLAM_H
#define SE3_GSLAM_H

#include <base/Types/SPtr.h>
#include <base/Types/SIM3.h>
#include <base/Thread/Thread.h>

namespace GSLAM{

typedef pi::UInt8   uchar;
typedef uchar       byte;
typedef std::size_t PointID;
typedef std::size_t FrameID;
typedef pi::Point2d Point2D;
typedef pi::Point2i Point2i;
typedef pi::Point3d Point3Type;
typedef pi::Point3ub ColorType;
typedef pi::SE3d    SE3;

}
#endif // SE3_H
