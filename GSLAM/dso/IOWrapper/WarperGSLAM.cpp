#include "WarperGSLAM.h"
#include "../util/FrameShell.h"
#include "../FullSystem/HessianBlocks.h"
#include <iostream>
using namespace std;
using namespace dso;
using namespace dso::IOWrap;

#include "ImageDisplay.h"
#include "ImageRW.h"


namespace dso
{
namespace IOWrap
{

void displayImage(const char* windowName, const MinimalImageB* img, bool autoSize ){}
void displayImage(const char* windowName, const MinimalImageB3* img, bool autoSize){}
void displayImage(const char* windowName, const MinimalImageF* img, bool autoSize ){}
void displayImage(const char* windowName, const MinimalImageF3* img, bool autoSize){}
void displayImage(const char* windowName, const MinimalImageB16* img, bool autoSize){}


void displayImageStitch(const char* windowName, const std::vector<MinimalImageB*> images, int cc, int rc){}
void displayImageStitch(const char* windowName, const std::vector<MinimalImageB3*> images, int cc, int rc){}
void displayImageStitch(const char* windowName, const std::vector<MinimalImageF*> images, int cc, int rc){}
void displayImageStitch(const char* windowName, const std::vector<MinimalImageF3*> images, int cc, int rc){}

int waitKey(int milliseconds){}
void closeAllWindows(){}

MinimalImageB* readImageBW_8U(std::string filename){}
MinimalImageB3* readImageRGB_8U(std::string filename){}
MinimalImage<unsigned short>* readImageBW_16U(std::string filename){}


MinimalImageB* readStreamBW_8U(char* data, int numBytes){}

void writeImage(std::string filename, MinimalImageB* img){}
void writeImage(std::string filename, MinimalImageB3* img){}
void writeImage(std::string filename, MinimalImageF* img){}
void writeImage(std::string filename, MinimalImageF3* img){}




}}

WarperGSLAM::WarperGSLAM(int width,int height)
    :w(width),h(height)
{
    settings_scaledVarTH=0.001;
    settings_absVarTH=0.001;
    settings_minRelBS=0.1;
    settings_pointCloudMode=1;
    settings_sparsity=1;
    settings_showCurrentCamera=true;
    settings_showTrajectory=true;
    settings_showFullTrajectory=true;
    settings_showAllConstraints=true;
    setting_render_display3D=true;
    currentCam=SPtr<KeyFrameDisplay>(new KeyFrameDisplay());
}

WarperGSLAM::~WarperGSLAM(){

}

void WarperGSLAM::publishGraph(const std::map<long,Eigen::Vector2i> &connectivity)
{
    if(!setting_render_display3D) return;

    model3DMutex.lock();
    connections.resize(connectivity.size()/2);
    int runningID=0;
    int totalActFwd=0, totalActBwd=0, totalMargFwd=0, totalMargBwd=0;
    for(std::pair<long,Eigen::Vector2i> p : connectivity)
    {
        int host = (int)(p.first >> 32);
        int target = (int)(p.first & (long)0xFFFFFFFF);

        assert(host >= 0 && target >= 0);
        if(host == target)
        {
            assert(p.second[0] == 0 && p.second[1] == 0);
            continue;
        }

        if(host > target) continue;

        connections[runningID].from = keyframesByKFID.count(host) == 0 ? 0 : keyframesByKFID[host];
        connections[runningID].to = keyframesByKFID.count(target) == 0 ? 0 : keyframesByKFID[target];
        connections[runningID].fwdAct = p.second[0];
        connections[runningID].fwdMarg = p.second[1];
        totalActFwd += p.second[0];
        totalMargFwd += p.second[1];

        long inverseKey = (((long)target) << 32) + ((long)host);
        Eigen::Vector2i st = connectivity.at(inverseKey);
        connections[runningID].bwdAct = st[0];
        connections[runningID].bwdMarg = st[1];

        totalActBwd += st[0];
        totalMargBwd += st[1];

        runningID++;
    }

    connections.resize(runningID);
    model3DMutex.unlock();
}

void WarperGSLAM::publishKeyframes(std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib)
{
    if(!setting_render_display3D) return;

    pi::ScopedMutex lk(model3DMutex);
    for(FrameHessian* fh : frames)
    {
        if(keyframesByKFID.find(fh->frameID) == keyframesByKFID.end())
        {
            SPtr<KeyFrameDisplay> kfd = SPtr<KeyFrameDisplay>(new KeyFrameDisplay());
            kfd->setFromKF(fh,HCalib);
            keyframesByKFID[fh->frameID] = kfd;
            keyframes.push_back(kfd);
        }
        else
            keyframesByKFID[fh->frameID]->setFromKF(fh, HCalib);
    }
}

void WarperGSLAM::publishCamPose(FrameShell* frame, CalibHessian* HCalib)
{
    pi::ScopedMutex lk(model3DMutex);

    currentCam->setFromF(frame, HCalib);

    allFramePoses.push_back(frame->camToWorld.translation().cast<float>());
    lastPose=frame->camToWorld;
}

void WarperGSLAM::pushLiveFrame(FrameHessian* image)
{
    if(!setting_render_displayVideo) return;
    pi::ScopedMutex lock(mutexImage);
    if(video.empty()) video=GSLAM::GImage(w,h,GSLAM::GImageType<u_char,3>::Type);
    for(int i=0;i<w*h;i++)
        video.at<pi::Point3ub>(i).x=
                video.at<pi::Point3ub>(i).y=
                video.at<pi::Point3ub>(i).z=
                image->dI[i][0]*0.8 > 255.0f ? 255.0 : image->dI[i][0]*0.8;


}

void WarperGSLAM::pushResidualImage(MinimalImageB3* image)
{
    cerr<<"pushResidualImage"<<endl;

}

void WarperGSLAM::pushDepthImage(MinimalImageB3* image)
{

}

void WarperGSLAM::join(){

}

void WarperGSLAM::reset(){

    keyframes.clear();
    keyframesByKFID.clear();
    connections.clear();
    allFramePoses.clear();
    currentCam=SPtr<KeyFrameDisplay>(new KeyFrameDisplay());
}

void WarperGSLAM::drawConstraints()
{

    if(settings_showAllConstraints)
    {
        // draw constraints
        glLineWidth(1);
        glBegin(GL_LINES);

        glColor3f(0,1,0);
        glBegin(GL_LINES);
        for(unsigned int i=0;i<connections.size();i++)
        {
            if(connections[i].to == 0 || connections[i].from==0) continue;
            int nAct = connections[i].bwdAct + connections[i].fwdAct;
            int nMarg = connections[i].bwdMarg + connections[i].fwdMarg;
            if(nAct==0 && nMarg>0  )
            {
                Sophus::Vector3f t = connections[i].from->camToWorld.translation().cast<float>();
                glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
                t = connections[i].to->camToWorld.translation().cast<float>();
                glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
            }
        }
        glEnd();
    }

    if(settings_showActiveConstraints)
    {
        glLineWidth(3);
        glColor3f(0,0,1);
        glBegin(GL_LINES);
        for(unsigned int i=0;i<connections.size();i++)
        {
            if(connections[i].to == 0 || connections[i].from==0) continue;
            int nAct = connections[i].bwdAct + connections[i].fwdAct;

            if(nAct>0)
            {
                Sophus::Vector3f t = connections[i].from->camToWorld.translation().cast<float>();
                glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
                t = connections[i].to->camToWorld.translation().cast<float>();
                glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
            }
        }
        glEnd();
    }

    if(settings_showTrajectory)
    {
        float colorRed[3] = {1,0,0};
        glColor3f(colorRed[0],colorRed[1],colorRed[2]);
        glLineWidth(3);

        glBegin(GL_LINE_STRIP);
        for(unsigned int i=0;i<keyframes.size();i++)
        {
            glVertex3f((float)keyframes[i]->camToWorld.translation()[0],
                    (float)keyframes[i]->camToWorld.translation()[1],
                    (float)keyframes[i]->camToWorld.translation()[2]);
        }
        glEnd();
    }

    if(settings_showFullTrajectory)
    {
        float colorGreen[3] = {0,1,0};
        glColor3f(colorGreen[0],colorGreen[1],colorGreen[2]);
        glLineWidth(3);

        glBegin(GL_LINE_STRIP);
        for(unsigned int i=0;i<allFramePoses.size();i++)
        {
            glVertex3f((float)allFramePoses[i][0],
                    (float)allFramePoses[i][1],
                    (float)allFramePoses[i][2]);
        }
        glEnd();
    }
}

void WarperGSLAM::draw()
{
    if(setting_render_display3D)
    {
        glDisable(GL_LIGHTING);
        // Activate efficiently by object
        pi::ScopedMutex lk3d(model3DMutex);
        //pangolin::glDrawColouredCube();
        int refreshed=0;
        for(SPtr<KeyFrameDisplay> fh : keyframes)
        {
            float blue[3] = {0,0,1};
            if(this->settings_showKFCameras)
                fh->drawCam(1,blue,0.1);


            refreshed =+ (int)(fh->refreshPC(refreshed < 10, this->settings_scaledVarTH, this->settings_absVarTH,
                    this->settings_pointCloudMode, this->settings_minRelBS, this->settings_sparsity));
            fh->drawPC(1);
        }
        if(this->settings_showCurrentCamera) currentCam->drawCam(2,0,0.2);

        drawConstraints();
    }

    if(!video.empty()&&video.type()==GSLAM::GImageType<unsigned char,3>::Type)
    {
        pi::ScopedMutex lock(mutexImage);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        int updownSwap = 1;
        u_char *buf = video.data;

        // FIXME: up/down swap
        if( updownSwap ) {
            int w, h, c, lw;
            u_char *p1, *p2;

            w = video.cols;
            h = video.rows;
            c = video.channels();
            lw = w*c;

            buf = new u_char[w*h*c];

            p1 = video.data;
            p2 = buf + (h-1)*lw;

            for(int j=0; j<h; j++) {
                memcpy(p2, p1, sizeof(char)*lw);

                p1 += lw;
                p2 -= lw;
            }
        }

        glDrawPixels(video.cols, video.rows,
                     GL_BGR, GL_UNSIGNED_BYTE,
                     buf);

        if( updownSwap ) delete [] buf;
        //        glViewport(0,0,win3d->width(),win3d->height());
    }
}
