#include <QAction>
#include <QTreeWidget>
#include <QTabWidget>
#include <QCloseEvent>

#include "QGLViewer/qglviewer.h"
#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/NodeGL.h>

namespace GSLAM{
class VisNodeGL;

class Win3D : public QGLViewer
{
    Q_OBJECT
public:
    Win3D(QWidget* parent,Svar config)
        : QGLViewer(parent){
        setObjectName("3D");
        setProperty("area","center");
        setProperty("stay",true);
        _pub_draw=messenger.advertise<Svar>("qviz/gl_draw");
        _pub_selected=messenger.advertise<int>("qviz/gl_selected");
        _sub_update=messenger.subscribe("qviz/gl_update",[this](bool update){
            emit signalUpdateGL();
        });
        _sub_radius=messenger.subscribe("qviz/gl_radius",[this](double radius){
            setSceneRadius(radius);
        });
        _sub_center=messenger.subscribe("qviz/gl_center",[this](Point3d c){
            setSceneCenter(qglviewer::Vec(c.x,c.y,c.z));
        });
        _sub_pose=messenger.subscribe("qviz/gl_camera_pose",[this](SE3 pose){
                GSLAM::Point3d t=pose.get_translation();
                GSLAM::SO3 r=pose.get_rotation();
                this->camera()->setPosition(qglviewer::Vec(t.x,t.y,t.z));
                this->camera()->setOrientation(qglviewer::Quaternion(r.w,r.z,-r.y,-r.x));
        });
        _sub_node=messenger.subscribe("qviz/gl_node",[this](Svar node){
            emit signalNode(node);
        });
        connect(this,SIGNAL(signalUpdateGL()),
                this,SLOT(updateGL()));
        connect(this,SIGNAL(signalNode(Svar)),
                this,SLOT(slotNode(Svar)));
    }
signals:
    void signalSetSceneRadius(qreal);
    void signalUpdateGL();
    void signalSetSceneCenter(qglviewer::Vec);
    void signalNode(Svar node);

public slots:
    void slotNode(Svar node);
public:
    void updateScenseCenterRadius();

    virtual void draw();

    virtual void fastDraw()
    {
        return draw();
        _status["fastDraw"]=true;
        _pub_draw.publish(_status);
    }

    virtual void initializeGL(){
        QGLViewer::initializeGL();
    }

    virtual bool setScenceRadius(float radius){
        QGLViewer::setSceneRadius(radius);
        return true;
    }

    bool setGLCenter(const GSLAM::Point3d& center){
        setSceneCenter(qglviewer::Vec(center.x,center.y,center.z));
        double radius=(camera()->position()-qglviewer::Vec(center.x,center.y,center.z)).norm();
//        LOG(INFO)<<"Radius is set to "<<radius;
        QGLViewer::setSceneRadius(radius);
        return true;
    }

    bool setViewPose(const GSLAM::SE3& pose){
        GSLAM::Point3d t=pose.get_translation();
        GSLAM::SO3 r=pose.get_rotation();
        camera()->setPosition(qglviewer::Vec(t.x,t.y,t.z));
        camera()->setOrientation(qglviewer::Quaternion(r.w,r.z,-r.y,-r.x));
        return true;
    }

    SE3 getCameraPose() {
        qglviewer::Camera* cam = camera();
        qglviewer::Vec v       = cam->position();
        qglviewer::Quaternion r = cam->orientation();
        return GSLAM::SE3(v.x,v.y,v.z,-r[3],-r[2],r[1],r[0]);
    }

    GSLAM::Camera getCamera(){
        qglviewer::Camera* cam = camera();
        float height = cam->screenHeight();
        float width = cam->screenWidth();
        float fx = width / (2 * ( tan(cam->horizontalFieldOfView() / 2) ) );
        float fy = height / (2 * ( tan(cam->fieldOfView()/2) ) );
        return GSLAM::Camera({width,height,fx,fy,width/2,height/2});
    }

    virtual void mousePressEvent(QMouseEvent* arg)
    {
        QMouseEvent* e=(QMouseEvent*)arg;
        if ((e->button() == Qt::LeftButton) && (e->modifiers() == Qt::ControlModifier))
        {
            qglviewer::Vec target;
            bool           found;
            target=camera()->pointUnderPixel(e->pos(),found);
            if(found)
            {
                setSelectRegionWidth(5);
                setSelectRegionHeight(5);
                select(e->pos());
                // Update display to show new selected objects
                update();
                setGLCenter(GSLAM::Point3d(target.x,target.y,target.z));
                return;
            }
        }
        else if(e->button() == Qt::LeftButton||e->button()==Qt::RightButton)
        {
            qglviewer::Vec target;
            bool           found;
            target=camera()->pointUnderPixel(e->pos(),found);
            if(found)
                setGLCenter(GSLAM::Point3d(target.x,target.y,target.z));
        }

        QGLViewer::mousePressEvent(arg);
    }

    void endSelection(const QPoint& point)
    {
        Q_UNUSED(point);

        // Flush GL buffers
        glFlush();

        // Get the number of objects that were seen through the pick matrix frustum. Reset GL_RENDER mode.
        GLint nbHits = glRenderMode(GL_RENDER);

        if (nbHits <= 0)
            setSelectedName(-1);
        else
        {
            // Interpret results: each object created 4 values in the selectBuffer().
            // selectBuffer[4*i+1] is the object minimum depth value, while selectBuffer[4*i+3] is the id pushed on the stack.
            // Of all the objects that were projected in the pick region, we select the closest one (zMin comparison).
            // This code needs to be modified if you use several stack levels. See glSelectBuffer() man page.
            GLuint zMin = (selectBuffer())[1];
            setSelectedName((selectBuffer())[3]);
            for (int i=1; i<nbHits; ++i)
                if ((selectBuffer())[4*i+1] < zMin)
                {
                    zMin = (selectBuffer())[4*i+1];
                    setSelectedName((selectBuffer())[4*i+3]);
                }
        }

        int slcted=selectedName();
        if(slcted<=0) return;
        _pub_selected.publish(slcted);
    }

    Publisher   _pub_draw,_pub_selected;
    Subscriber  _sub_update,_sub_radius,_sub_center,_sub_pose,_sub_node;
    Svar        _status;

    Svar  nodevis;
};

}
