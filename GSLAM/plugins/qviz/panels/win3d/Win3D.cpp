#if QT_VERSION>=0x050000
#include <QtOpenGL/QGLFunctions>
#include <QOpenGLFunctions_3_3_Core>
#else
#include <GL/glew.h>
#endif

#include "Win3D.h"


namespace GSLAM {
void glMultMatrix(const SIM3& pose)
{
    const double A360byPI=114.59155902616439;
    Point3d trans=pose.get_translation();
    glTranslated(trans.x,trans.y,trans.z);
    double x,y,z,w;
    pose.get_rotation().getValue(x,y,z,w);
    glRotated(A360byPI*acos(w),x,y,z);
    glScaled(pose.get_scale(),pose.get_scale(),pose.get_scale());
}

class VisNodeGL: public QObject{
public:
    VisNodeGL(QGLViewer* parent,NodeGLPtr node)
        : QObject(parent),_vertexBuffer(0){
        setNode(node);
    }

    virtual ~VisNodeGL(){
        glDeleteBuffers(4, &_vertexBuffer);
    }

    void setNode(NodeGLPtr node){
//        _node=node;
        _name=node->name;
        if(!_vertexBuffer){
            glewInit();
            glGenBuffers(4, &_vertexBuffer);
            if(node->colors.size())
                defColor=node->colors.front();
        }
        bufferData(node);
    }

    void bufferData(NodeGLPtr node){
        _colorSize=node->colors.size();
        _vertexSize=node->vertices.size();
        _normalSize=node->normals.size();
        _faceSize=node->faces.size();
        _mode=node->mode();
        clicked_func=node->clicked_func;
        _box=node->boundingbox();
        _transform=node->transform;

        glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER,node->vertices.size()*sizeof(NodeGL::Vertex3f),
                     node->vertices.data(), GL_STATIC_DRAW);

        if(node->colors.size()){
            glBindBuffer(GL_ARRAY_BUFFER,_colorBuffer);
            glBufferData(GL_ARRAY_BUFFER,node->colors.size()*3*sizeof(NodeGL::Color3b),
                         node->colors.data(), GL_STATIC_DRAW);
        }

        if(node->normals.size()){
            glBindBuffer(GL_ARRAY_BUFFER,_normalBuffer);
            glBufferData(GL_ARRAY_BUFFER,node->normals.size()*3*sizeof(NodeGL::Vertex3f),
                         node->normals.data(), GL_STATIC_DRAW);
        }

        if(node->faces.size()){
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,_faceBuffer);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER,node->faces.size()*sizeof(unsigned int),
                         node->faces.data(), GL_STATIC_DRAW);
        }
    }

    void drawPoints(){
        glPointSize(pointSize);
        glDrawArrays(GL_POINTS,0,_vertexSize);
    }

    void drawLines(){
        glLineWidth(lineWidth);
        glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
        glDrawArrays(GL_LINES,0,_vertexSize);
    }

    void drawLineStrip(){
        glLineWidth(lineWidth);
        glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
        glDrawArrays(GL_LINE_STRIP,0,_vertexSize);
    }

    void drawMesh(){
        glPolygonMode(GL_FRONT, GL_FILL);
        glPolygonMode(GL_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, _faceSize, GL_UNSIGNED_INT,0);
    }

    void draw(){
        if(_transform.get_scale()<=0)
            return drawShape();
        glPushMatrix();
        glMultMatrix(_transform);
        drawShape();
        glPopMatrix();
    }

    void drawShape(){
        glBindBuffer(GL_ARRAY_BUFFER,_vertexBuffer);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        if(_colorSize==_vertexSize){
            glBindBuffer(GL_ARRAY_BUFFER,_colorBuffer);
            glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
        }
        else{
            glColor3ub(defColor.x,defColor.y,defColor.z);
        }

        if(_normalSize==_vertexSize){
            glEnable(GL_LIGHTING);
            glBindBuffer(GL_ARRAY_BUFFER,_normalBuffer);
            glNormalPointer(GL_UNSIGNED_BYTE, 0, 0);
        }
        else
            glDisable(GL_LIGHTING);

        glEnableClientState(GL_VERTEX_ARRAY);

        if(_colorSize==_vertexSize) {
            glEnableClientState(GL_COLOR_ARRAY);
        }

        if(_normalSize==_vertexSize){
            glEnableClientState(GL_NORMAL_ARRAY);
        }

        switch (_mode) {
        case NodeGL::POINTS:
            drawPoints();
            break;
        case NodeGL::LINES:
            drawLines();
            break;
        case NodeGL::LINE_STRIP:
            drawLineStrip();
            break;
        case NodeGL::MESH:
            drawMesh();
            break;
        default:
            break;
        }

        if(_colorSize==_vertexSize) {
            glDisableClientState(GL_COLOR_ARRAY);
        }

        if(_normalSize==_vertexSize){
            glDisableClientState(GL_NORMAL_ARRAY);
        }

        glDisableClientState(GL_VERTEX_ARRAY);
    }

    std::string  _name;

    unsigned int _vertexBuffer,_colorBuffer,_normalBuffer,_faceBuffer;
    size_t       _vertexSize,_colorSize,_normalSize,_faceSize;
    int          _mode;
    SIM3         _transform;
    std::pair<NodeGL::Vertex3f,NodeGL::Vertex3f> _box;

    std::function<bool(Point3d)> clicked_func;

    float    pointSize=2.5,lineWidth=2;
    Point3ub defColor =Point3ub(255,0,0);

    bool  visible=true;
};


void Win3D::slotNode(Svar msg)
{
    NodeGLPtr node=msg.castAs<NodeGLPtr>();
    if(!node) return;
    bool fitView=false;
    if(nodevis.isUndefined()){
        nodevis["__name__"]="NodeGL Visualizer";
        nodevis["__menu__"]["Delete"]=SvarFunction([this](){
            this->nodevis=Svar();
            updateGL();
        });
        fitView=true;
    }


    if(node->empty()) {
        nodevis.erase(node->name);
        messenger.publish("qviz/display",nodevis);
        updateGL();
        return;
    }

    Svar it=nodevis[node->name];
    if(it.isUndefined()){
        std::shared_ptr<VisNodeGL> ptr=std::make_shared<VisNodeGL>(this,node);
        Svar config;
        config["__holder__"]=ptr;
        if(ptr->_mode==NodeGL::POINTS)
        config.arg<double>("point_size",2.5,"The pointsize");
        if(ptr->_mode==NodeGL::LINES||ptr->_mode==NodeGL::LINE_STRIP)
        config.arg<double>("line_width",2.,"The line width to draw");
        config.arg<Point3ub>("default_color",ptr->defColor,"the default color");
        config.arg<bool>("visible",ptr->visible,"Visiable or not");
        config["__cbk__point_size"]=SvarFunction([ptr,this](){
            ptr->pointSize=nodevis[ptr->_name].get<double>("point_size",2.5);
            updateGL();
        });
        config["__cbk__line_width"]=SvarFunction([ptr,this](){
            ptr->lineWidth=nodevis[ptr->_name].get<double>("line_width",2.);
            updateGL();
        });
        config["__cbk__default_color"]=SvarFunction([ptr,this](){
            ptr->defColor=nodevis[ptr->_name].get<Point3ub>("default_color",Point3ub(255,0,0));
            updateGL();
        });
        config["__cbk__visible"]=SvarFunction([ptr,this](){
            ptr->visible=nodevis[ptr->_name].get("visible",ptr->visible);
            updateGL();
        });
        config["__name__"]=ptr->_name;

        nodevis[node->name]=config;
        messenger.publish("qviz/display",nodevis);
    }
    else
        nodevis[node->name]["__holder__"].castAs<std::shared_ptr<VisNodeGL>>()->setNode(node);

    updateScenseCenterRadius();
    if(fitView)
        QGLViewer::camera()->showEntireScene();
}


void Win3D::updateScenseCenterRadius()
{
    if(nodevis.size()<1) return;
    NodeGL::Vertex3f minV(std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max());
    NodeGL::Vertex3f maxV(-std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max(),
                 -std::numeric_limits<float>::max());

    for(auto it:nodevis.castAs<std::map<std::string,Svar>>()){
        if(it.first.front()=='_') continue;
        std::shared_ptr<VisNodeGL> vis=it.second["__holder__"].castAs<std::shared_ptr<VisNodeGL>>();
        if(vis->_vertexSize<1) continue;
        std::vector<Point3f> vertices={vis->_box.first,vis->_box.second};
        if(vis->_transform.get_scale()>0){
            std::vector<Point3f> add;
            for(int i=0;i<8;i++)
                add.push_back(Point3f(vertices[i&1].x,vertices[i&2].y,vertices[i&4].z));
            vertices=add;
        }

        for(NodeGL::Vertex3f v:vertices){
          if(vis->_transform.get_scale()>0)
              v=vis->_transform*v;
          minV.x=std::min(v.x,minV.x);
          minV.y=std::min(v.y,minV.y);
          minV.z=std::min(v.z,minV.z);
          maxV.x=std::max(v.x,maxV.x);
          maxV.y=std::max(v.y,maxV.y);
          maxV.z=std::max(v.z,maxV.z);
        }
    }
    setGLCenter((minV+maxV)*0.5);
    setScenceRadius((maxV-minV).norm()/2);
    updateGL();
}

void Win3D::draw()
{
    _status["fastDraw"]=false;
    _pub_draw.publish(_status);
    if(!nodevis.isObject()) return;
    for(auto it:nodevis.castAs<std::map<std::string,Svar>>()){
            if(it.first.front()=='_') continue;
            std::shared_ptr<VisNodeGL> vis=it.second["__holder__"].castAs<std::shared_ptr<VisNodeGL>>();
            if(vis->visible)
                vis->draw();
    }
}

}
GSLAM_REGISTER_PANEL(win3d,GSLAM::Win3D);
