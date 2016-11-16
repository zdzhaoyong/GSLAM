#ifndef KEYFRAMEDISPLAY_H
#define KEYFRAMEDISPLAY_H
#include <sstream>
#include <fstream>

#include <Eigen/Core>
#include "../util/NumType.h"
#include <gui/gl/OpenGL.h>

namespace dso
{

class CalibHessian;
class FrameHessian;
class FrameShell;

namespace IOWrap
{

template<int ppp>
struct InputPointSparse
{
    float u;
    float v;
    float idpeth;
    float idepth_hessian;
    float relObsBaseline;
    int numGoodRes;
    unsigned char color[ppp];
    unsigned char status;
};

struct MyVertex
{
    float point[3];
    unsigned char color[4];
};

// stores a pointcloud associated to a Keyframe.
class KeyFrameDisplay
{

public:
    KeyFrameDisplay();
    ~KeyFrameDisplay();

    // copies points from KF over to internal buffer,
    // keeping some additional information so we can render it differently.
    void setFromKF(FrameHessian* fh, CalibHessian* HCalib);

    // copies points from KF over to internal buffer,
    // keeping some additional information so we can render it differently.
    void setFromF(FrameShell* fs, CalibHessian* HCalib);

    // copies & filters internal data to GL buffer for rendering. if nothing to do: does nothing.
    bool refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity);

    // renders cam & pointcloud.
    void drawCam(float lineWidth = 1, float* color = 0, float sizeFactor=1);
    void drawPC(float pointSize);

    int id;
    bool active;
    SE3 camToWorld;

    inline bool operator < (const KeyFrameDisplay& other) const
    {
        return (id < other.id);
    }


private:
    float fx,fy,cx,cy;
    float fxi,fyi,cxi,cyi;
    int width, height;

    float my_scaledTH, my_absTH, my_scale;
    int my_sparsifyFactor;
    int my_displayMode;
    float my_minRelBS;
    bool needRefresh;


    int numSparsePoints;
    int numSparseBufferSize;
    InputPointSparse<28>* originalInputSparse;


    bool bufferValid;
    int numGLBufferPoints;
    int numGLBufferGoodPoints;

    GLuint  vertexBuffer,colorBuffer;
};

}
}

#endif // KEYFRAMEDISPLAY_H
