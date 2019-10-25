#ifndef GSLAM_NODEGL_H
#define GSLAM_NODEGL_H
#include <vector>
#include <limits>
#include <functional>
#include <memory>

#include "SIM3.h"
#include "GImage.h"

namespace GSLAM{

class NodeGL{
public:
  NodeGL(const std::string& name)
    : name(name),transform(SE3(),-1),displayMode(AUTO){}

  typedef Point3f  Vertex3f;
  typedef Point2f  Vertex2f;
  typedef Point3ub Color3b;
  typedef unsigned char uchar;

  std::string name;
  SIM3        transform;
  int         displayMode;

  std::vector<Vertex3f>       vertices,normals;
  std::vector<Color3b>        colors;
  std::vector<Vertex2f>       texcoords;
  std::vector<unsigned int>   faces,materials;
  std::vector<GSLAM::GImage>  textures;

  std::function<bool(Point3d)> clicked_func;

  enum DisplayMode
  {
    AUTO  =0,
    POINTS=1,     /// vertices [normals,colors]
    LINES =2,     /// vertices
    MESH  =3,     /// vertices,faces ...
    LINE_STRIP=4
  };

  bool empty()const{return vertices.empty();}

  std::pair<Vertex3f,Vertex3f> boundingbox()const{
    Vertex3f min(std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max());
    Vertex3f max(-std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max());

    for(const Vertex3f& v:vertices){
      min.x=std::min(v.x,min.x);
      min.y=std::min(v.y,min.y);
      min.z=std::min(v.z,min.z);
      max.x=std::max(v.x,max.x);
      max.y=std::max(v.y,max.y);
      max.z=std::max(v.z,max.z);
    }

    return std::make_pair(min,max);
  }

  int mode()const{
    if(displayMode!=AUTO) return displayMode;
    if(faces.size()) return MESH;
    return POINTS;
  }

};

typedef std::shared_ptr<NodeGL> NodeGLPtr;

}

#endif
