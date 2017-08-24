#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct InvDepthRelativePoseResiduals
{
    InvDepthRelativePoseResiduals(double* _match):match(_match){}

    template <typename T> bool
    operator()
    (
            const T* const iDepth,// inverse depth of x1,y1,z1
            const T* const Rt,    // rx,ry,rz,rw,x,y,z
            T* residuals          // dx,dy
            )
    const
    {
        const T   p1[3]={(T)match[0],(T)match[1],(T)match[2]};
        const T * t =&Rt[0];
        const T * R =&Rt[3];

        T p[3];
        // Rotate the point according the camera rotation
        ceres::AngleAxisRotatePoint(R, p1, p);

        // Apply the camera translation
        p[0] += t[0]*iDepth[0];
        p[1] += t[1]*iDepth[0];
        p[2] += t[2]*iDepth[0];

        const T invW=1./p[2];
        residuals[0]= invW*p[0]-T(match[3]/match[5]);
        residuals[1]= invW*p[1]-T(match[4]/match[5]);
        (*(double*)&residuals[0])*=match[5];
        (*(double*)&residuals[1])*=match[5];
        return true;
    }

    static int num_residuals() { return 2;}

    double*     match;// x1,y1,z1,x2,y2,z2
};

class InvDepthRelativePoseCostFunction: public ceres::AutoDiffCostFunction<InvDepthRelativePoseResiduals, 2, 1, 6>
{
public:
    InvDepthRelativePoseCostFunction(double* match)
        :ceres::AutoDiffCostFunction<InvDepthRelativePoseResiduals, 2, 1, 6>
         (new InvDepthRelativePoseResiduals(match))
    {

    }
};

struct IdeaProjectResiduals
{
    IdeaProjectResiduals(const double* const pCamera,const double* const pWorld=NULL)
        :Pc(pCamera),Pw(pWorld)
    {

    }

    template <typename T> bool
    operator()
    (
            const T* const Rt, // x,y,z,rx,ry,rz ---- DOF:6
            T* residuals       // dx,dy ---- DOF:2
            )
    const
    {
        const T * t =&Rt[0];
        const T * R =&Rt[3];
        T p[3];
        // Rotate the point according the camera rotation
        const T   p1[3]={(T)Pw[0],(T)Pw[1],(T)Pw[2]};
        ceres::AngleAxisRotatePoint(R, p1, p);

        // Apply the camera translation
        p[0] += t[0];
        p[1] += t[1];
        p[2] += t[2];

        const T invW=T(1.)/p[2];
        residuals[0]= invW*p[0]-Pc[0]/Pc[2];
        residuals[1]= invW*p[1]-Pc[1]/Pc[2];
        (*(double*)&residuals[0])*=Pc[2];
        (*(double*)&residuals[1])*=Pc[2];
        return true;
    }

    template <typename T> bool
    operator()
    (
            const T* const Pw, // X,Y,Z world position of the Point ---- DOF:3
            const T* const Rt, // x,y,z,rx,ry,rz ---- DOF:6
            T* residuals       // dx,dy ---- DOF:2
            )
    const
    {
        const T * t =&Rt[0];
        const T * R =&Rt[3];
        T p[3];
        // Rotate the point according the camera rotation
        ceres::AngleAxisRotatePoint(R, Pw, p);

        // Apply the camera translation
        p[0] += t[0];
        p[1] += t[1];
        p[2] += t[2];

        const T invW=T(1.)/p[2];
        residuals[0]= invW*p[0]-Pc[0]/Pc[2];
        residuals[1]= invW*p[1]-Pc[1]/Pc[2];
        (*(double*)&residuals[0])*=Pc[2];
        (*(double*)&residuals[1])*=Pc[2];
        return true;
    }

    static int num_residuals() { return 2;}

    static ceres::CostFunction* create(const double* const pCamera,const double* const pWorld=NULL)
    {
        if(pWorld)
        {
            return new ceres::AutoDiffCostFunction<IdeaProjectResiduals,2,6>(new IdeaProjectResiduals(pCamera,pWorld));// Optimize both mappoints and keyframe
        }
        else
            return new ceres::AutoDiffCostFunction<IdeaProjectResiduals,2,3,6>(new IdeaProjectResiduals(pCamera));// Only optimize keyframe poses
    }

    const double* const Pc;// Projection on z=1 plane ---- DOF:2
    const double* const Pw;// X,Y,Z world position of the Point ---- DOF:3
};

class IdeaCameraPnPCostFunction : public ceres::AutoDiffCostFunction<IdeaProjectResiduals,2,6>
{
public:
    IdeaCameraPnPCostFunction(double* match)
        : ceres::AutoDiffCostFunction<IdeaProjectResiduals,2,6>(new IdeaProjectResiduals(match+3,match))
    {}
};

struct EdgePointICPSim3Residuals
{
public:
    EdgePointICPSim3Residuals(double* match)
        :src(match[3],match[4],match[5]),dst(match[0],match[1],match[2])
    {}
    template <typename T> bool
    operator()
    (
            const T* const Rt,    // x,y,z,rx,ry,rz,scale
            T* residuals          // dx,dy,dz
            )
    const
    {
        T rs[3];
        T srcVec[3]={T(src(0)),T(src(1)),T(src(2))};
        ceres::AngleAxisRotatePoint(Rt+3, srcVec, rs);
        residuals[0]=rs[0]*Rt[6]+Rt[0]-T(dst(0));
        residuals[1]=rs[1]*Rt[6]+Rt[1]-T(dst(1));
        residuals[2]=rs[2]*Rt[6]+Rt[2]-T(dst(2));
        return true;
    }

    static int num_residuals() { return 3;}
private:
    const Eigen::Vector3d src,dst;
};

class EdgePointICPSim3CostFunction : public ceres::AutoDiffCostFunction<EdgePointICPSim3Residuals,3,7>
{
public:
    EdgePointICPSim3CostFunction(double* match):
        ceres::AutoDiffCostFunction<EdgePointICPSim3Residuals,3,7>(new EdgePointICPSim3Residuals(match)){}
};

struct IdeaProjectMapPointSIM3BundleResiduals
{
    IdeaProjectMapPointSIM3BundleResiduals(const double* const pCamera)
        :Pc(pCamera)
    {
    }

    template <typename T> bool
    operator()
    (
            const T* const Pw, // X,Y,Z world position of the Point ---- DOF:3
            const T* const Rt, // x,y,z,rx,ry,rz,scale ---- DOF:7
            T* residuals       // dx,dy ---- DOF:2
            )
    const
    {
        const T * t =&Rt[0];
        const T * R =&Rt[3];
        T p[3];
        T invScale=T(1.)/Rt[6];
        p[0]=(Pw[0]-t[0])*invScale;
        p[1]=(Pw[1]-t[1])*invScale;
        p[2]=(Pw[2]-t[2])*invScale;
        T invR[3]={-R[0],-R[1],-R[2]};
        // Rotate the point according the camera rotation
        ceres::AngleAxisRotatePoint(invR, p, p);

        const T invW=T(1.)/p[2];
        residuals[0]= invW*p[0]-Pc[0]/Pc[2];
        residuals[1]= invW*p[1]-Pc[1]/Pc[2];
        (*(double*)&residuals[0])*=Pc[2];
        (*(double*)&residuals[1])*=Pc[2];
        return true;
    }

    static int num_residuals() { return 2;}

    const double* const Pc;// Projection on z=1 plane ---- DOF:2
};

class IdeaProjectMapPointSIM3BundleCostFunction : public ceres::AutoDiffCostFunction<IdeaProjectMapPointSIM3BundleResiduals,2,3,7>
{
public:
    IdeaProjectMapPointSIM3BundleCostFunction(const double* const pCamera):
        ceres::AutoDiffCostFunction<IdeaProjectMapPointSIM3BundleResiduals,2,3,7>(new IdeaProjectMapPointSIM3BundleResiduals(pCamera)){}
};

struct PinholeProjectMapPointSIM3BundleResiduals
{
    PinholeProjectMapPointSIM3BundleResiduals(const double* const pCamera)
        :Pc(pCamera){

    }

    template <typename T> bool
    operator()
    (
            const T* const Pw, // X,Y,Z world position of the Point ---- DOF:3
            const T* const Rt, // x,y,z,rx,ry,rz ---- DOF:6
            const T* const fc, // focal,centerX,centerY
            T* residuals       // dx,dy ---- DOF:2
            )
    const
    {
        const T * t =&Rt[0];
        const T * R =&Rt[3];
        const T * f =&fc[0];
        const T * cx=&fc[1];
        const T * cy=&fc[2];
        T p[3];
        T invScale=T(1.)/Rt[6];
        p[0]=(Pw[0]-t[0])*invScale;
        p[1]=(Pw[1]-t[1])*invScale;
        p[2]=(Pw[2]-t[2])*invScale;
        T invR[3]={-R[0],-R[1],-R[2]};
        // Rotate the point according the camera rotation
        ceres::AngleAxisRotatePoint(invR, p, p);

        residuals[0]= p[0]*f/p[2]-Pc[0]/Pc[2]+cx;
        residuals[1]= p[1]*f/p[2]-Pc[1]/Pc[2]+cy;
        (*(double*)&residuals[0])*=Pc[2];
        (*(double*)&residuals[1])*=Pc[2];
        return true;
    }

    static int num_residuals() { return 2;}
    const double* const Pc; // Projection on z=1 plane ---- DOF:2
};

struct IdeaProjectInvDepthSIM3BundleResiduals
{
    IdeaProjectInvDepthSIM3BundleResiduals(const double* const pCameraHost,const double* const pCameraChild)
        :Pch(pCameraHost),Pcc(pCameraChild)
    {}

    template <typename T>
    inline void SIM3Mult(const T* Rl,const T* tl,const T& scaleL,
                        const T* Rr,const T* tr,const T& scaleR,
                        T* R,T* t,T& scale)const
    {
        ceres::QuaternionProduct(Rl,Rr,R);
        ceres::UnitQuaternionRotatePoint(Rl,tr,t);
        for(int i=0;i<3;i++)
            t[i]=t[i]*scaleL+tl[i];
        scale=scaleL*scaleR;
    }

    template <typename T>
    inline void SIM3Mult(const T* Rtl,const T* Rtr,T* Rt)const
    {
        SIM3Mult(Rtl,Rtl+4,Rtl[7],Rtr,Rtr+4,Rtr[7],Rt,Rt+4,Rt[7]);
    }

    template <typename T> bool
    operator()
    (
            const T* const HostRt, // x,y,z,rx,ry,rz,scale ---- DOF:7
            const T* const ChildRt,// x,y,z,rx,ry,rz,scale ---- DOF:7
            const T* const Idepth, // inv depth estimation ---- DOF:1
            T* residuals           // dx,dy ---- DOF:2
            )
    const
    {
        // compute host to child SIM3 Tch=Tc^-1*Th
        T quatH[4],quatC[4];
        ceres::AngleAxisToQuaternion(HostRt+3,quatH);
        ceres::AngleAxisToQuaternion(ChildRt+3,quatC);
        // compute inverse ChildRt
        for(int i=1;i<4;i++) quatC[i]=-quatC[i];
        T transinvTc[4];
        ceres::UnitQuaternionRotatePoint(quatC,ChildRt,transinvTc);
        transinvTc[3]=T(1.)/ChildRt[6];
        for(int i=0;i<3;i++)
            transinvTc[i]=-transinvTc[i]*transinvTc[3];

        T Tch[8];
        SIM3Mult(quatC,transinvTc,transinvTc[3],quatH,HostRt,HostRt[6],Tch,Tch+4,Tch[7]);
        const T   p1[3]={Tch[7]*(T)Pch[0],Tch[7]*(T)Pch[1],Tch[7]*(T)Pch[2]};
        T p[3];
        ceres::UnitQuaternionRotatePoint(Tch,p1,p);
        // Apply the camera translation
        p[0] += Tch[4]*Idepth[0];
        p[1] += Tch[5]*Idepth[0];
        p[2] += Tch[6]*Idepth[0];

        residuals[0]= p[0]/p[2]-T(Pcc[0]/Pcc[2]);
        residuals[1]= p[1]/p[2]-T(Pcc[1]/Pcc[2]);
        (*(double*)&residuals[0])*=Pcc[2];
        (*(double*)&residuals[1])*=Pcc[2];
        return true;
    }

    static int num_residuals() { return 2;}

    const double* const Pch;// Projection on z=1 plane ---- DOF:2
    const double* const Pcc;// Projection on z=1 plane ---- DOF:2
};

class IdeaProjectInvDepthSIM3BundleCostFunction : public ceres::AutoDiffCostFunction<IdeaProjectInvDepthSIM3BundleResiduals,2,7,7,1>
{
public:
    IdeaProjectInvDepthSIM3BundleCostFunction(const double* const pCameraHost,const double* const pCameraChild):
        ceres::AutoDiffCostFunction<IdeaProjectInvDepthSIM3BundleResiduals,2,7,7,1>(new IdeaProjectInvDepthSIM3BundleResiduals(pCameraHost,pCameraChild)){}
};

struct EdgeSE3Residuals
{
public:
    EdgeSE3Residuals(double* quat_trans,double *information)
        :T12(quat_trans),info(information)
    {
        if(info)
        {
            Eigen::Map<Eigen::Matrix<double, 6, 6> > mapInfo(information);
            Eigen::LLT<Eigen::Matrix<double, 6, 6>>  llt(mapInfo);
            sqrt_info = llt.matrixL();
        }
    }

    template <typename T>
    inline void SE3Mult(const T* Rl,const T* tl,
                        const T* Rr,const T* tr,
                        T* R,T* t)const
    {
        ceres::QuaternionProduct(Rl,Rr,R);
        ceres::UnitQuaternionRotatePoint(Rl,tr,t);
        for(int i=0;i<3;i++)
            t[i]=t[i]+tl[i];
    }

    template <typename T>
    inline void SE3Mult(const T* Rtl,const T* Rtr,T* Rt)const
    {
        SE3Mult(Rtl,Rtl+4,Rtr,Rtr+4,Rt,Rt+4);
    }

    template <typename T> bool
    operator()
    (
            const T* const Rt1,    // x,y,z,rx,ry,rz
            const T* const Rt2,
            T* residuals          // dx,dy,dz,drx,dry,drz error=T12*T2^-1*T1
            )
    const
    {
        T quat1[4],quat2[4];
        ceres::AngleAxisToQuaternion(Rt1+3,quat1);
        ceres::AngleAxisToQuaternion(Rt2+3,quat2);
        for(int i=1;i<4;i++) quat2[i]=-quat2[i];
        T trans2inv[3];
        ceres::UnitQuaternionRotatePoint(quat2,Rt2,trans2inv);
        for(int i=0;i<3;i++) trans2inv[i]=-trans2inv[i];

        T trans12[7]={T(T12[3]),T(T12[0]),T(T12[1]),T(T12[2]),T(T12[4]),T(T12[5]),T(T12[6])};//qx,qy,qz,qw,x,y,z
        T estT21[7],error[7];
        SE3Mult(quat2,trans2inv,quat1,Rt1,estT21,estT21+4);
        SE3Mult(trans12,estT21,error);
        ceres::QuaternionToAngleAxis(error,residuals+3);
        residuals[0]=error[4];
        residuals[1]=error[5];
        residuals[2]=error[6];

        if(info)
        {
            T* r=residuals;
            double  res[6]={0,0,0,0,0,0};
            const Eigen::Matrix<double,6,6>& si=sqrt_info;
            for(int i=0;i<num_residuals();i++)
                for(int j=0;j<num_residuals();j++)
                    res[i]+=si(i,j)*(*(double*)&r[j]);
            for(int i=0;i<num_residuals();i++)
                *(double*)(residuals+i)=res[i];
        }

        return true;
    }
    bool
    operator()
    (
            const ceres::Jet<double,12>* const Rt1,    // x,y,z,rx,ry,rz
            const ceres::Jet<double,12>* const Rt2,
            ceres::Jet<double,12>* residuals          // dx,dy,dz,drx,dry,drz error=T12*T2^-1*T1
            )
    const
    {
        typedef ceres::Jet<double,12> T;
        T quat1[4],quat2[4];
        ceres::AngleAxisToQuaternion(Rt1+3,quat1);
        ceres::AngleAxisToQuaternion(Rt2+3,quat2);
        for(int i=1;i<4;i++) quat2[i]=-quat2[i];
        T trans2inv[3];
        ceres::UnitQuaternionRotatePoint(quat2,Rt2,trans2inv);
        for(int i=0;i<3;i++) trans2inv[i]=-trans2inv[i];

        T trans12[7]={T(T12[3]),T(T12[0]),T(T12[1]),T(T12[2]),T(T12[4]),T(T12[5]),T(T12[6])};//qx,qy,qz,qw,x,y,z
        T estT21[7],error[7];
        SE3Mult(quat2,trans2inv,quat1,Rt1,estT21,estT21+4);
        SE3Mult(trans12,estT21,error);
        ceres::QuaternionToAngleAxis(error,residuals+3);
        residuals[0]=error[4];
        residuals[1]=error[5];
        residuals[2]=error[6];
        return true;
    }
    static int num_residuals() { return 6;}
private:
    const double* T12;//SE3_{12}:=SE3_1^{-1}*SE3_2
    const double* info;
    Eigen::Matrix<double,6,6> sqrt_info;
};

//class EdgeSE3CostFunction : public ceres::AutoDiffCostFunction<EdgeSE3Residuals,6,6,6>
//{
//public:
//    EdgeSE3CostFunction(double* quat_trans,double *info):
//        ceres::AutoDiffCostFunction<EdgeSE3Residuals,6,6,6>(new EdgeSE3Residuals(quat_trans,info)){}
//};

class EdgeSE3CostFunction : public ceres::AutoDiffCostFunction<EdgeSE3Residuals,6,7,7>
{
public:
    EdgeSE3CostFunction(double* quat_trans,double *info):
        ceres::AutoDiffCostFunction<EdgeSE3Residuals,6,7,7>(new EdgeSE3Residuals(quat_trans,info)){}
};

struct EdgeSIM3Residuals
{
public:
    EdgeSIM3Residuals(double* quat_trans,double *information)
        :T12(quat_trans),info(information)
    {
        if(info)
        {
            Eigen::Map<Eigen::Matrix<double, 7, 7> > mapInfo(information);
            Eigen::LLT<Eigen::Matrix<double, 7, 7>>  llt(mapInfo);
            sqrt_info = llt.matrixL();
        }
    }

    template <typename T>
    inline void SIM3Mult(const T* Rl,const T* tl,const T& scaleL,
                        const T* Rr,const T* tr,const T& scaleR,
                        T* R,T* t,T& scale)const
    {
        ceres::QuaternionProduct(Rl,Rr,R);
        ceres::UnitQuaternionRotatePoint(Rl,tr,t);
        for(int i=0;i<3;i++)
            t[i]=t[i]*scaleL+tl[i];
        scale=scaleL*scaleR;
    }

    template <typename T>
    inline void SIM3Mult(const T* Rtl,const T* Rtr,T* Rt)const
    {
        SIM3Mult(Rtl,Rtl+4,Rtl[7],Rtr,Rtr+4,Rtr[7],Rt,Rt+4,Rt[7]);
    }

    template <typename T> bool
    operator()
    (
            const T* const Rt1,    // x,y,z,rx,ry,rz
            const T* const Rt2,
            T* residuals          // dx,dy,dz,drx,dry,drz error=T12*T2^-1*T1
            )
    const
    {
        T quat1[4],quat2[4];
        ceres::AngleAxisToQuaternion(Rt1+3,quat1);
        ceres::AngleAxisToQuaternion(Rt2+3,quat2);
        // compute inverse T2
        for(int i=1;i<4;i++) quat2[i]=-quat2[i];
        T trans2inv[4];
        trans2inv[3]=T(1.)/Rt2[6];
        ceres::UnitQuaternionRotatePoint(quat2,Rt2,trans2inv);
        for(int i=0;i<3;i++)
            trans2inv[i]=-trans2inv[i]*trans2inv[3];

        T trans12[8]={T(T12[3]),T(T12[0]),T(T12[1]),T(T12[2]),T(T12[4]),T(T12[5]),T(T12[6]),T(T12[7])};//qw,qx,qy,qz,x,y,z,scale
        T estT21[8],error[8];
        SIM3Mult(quat2,trans2inv,trans2inv[3],quat1,Rt1,Rt1[6],estT21,estT21+4,estT21[7]);
        SIM3Mult(trans12,estT21,error);
        ceres::QuaternionToAngleAxis(error,residuals+3);
        residuals[0]=error[4];
        residuals[1]=error[5];
        residuals[2]=error[6];
        residuals[6]=error[7];

        if(info)
        {
            T* r=residuals;
            double  res[7]={0,0,0,0,0,0,0};
            const Eigen::Matrix<double,7,7>& si=sqrt_info;
            for(int i=0;i<num_residuals();i++)
                for(int j=0;j<num_residuals();j++)
                    res[i]+=si(i,j)*(*(double*)&r[j]);
            for(int i=0;i<num_residuals();i++)
                *(double*)(residuals+i)=res[i];
        }
        return true;
    }

    static int num_residuals() { return 7;}
private:
    const double* T12;//SE3_{12}:=SE3_1^{-1}*SE3_2  qx,qy,qz,qw,x,y,z,scale
    const double* info;
    Eigen::Matrix<double,7,7> sqrt_info;
};

class EdgeSIM3CostFunction : public ceres::AutoDiffCostFunction<EdgeSIM3Residuals,7,7,7>
{
public:
    EdgeSIM3CostFunction(double* quat_trans,double *info):
        ceres::AutoDiffCostFunction<EdgeSIM3Residuals,7,7,7>(new EdgeSIM3Residuals(quat_trans,info)){}
};

struct EdgeGPSResiduals
{
public:
    EdgeGPSResiduals(double* quat_trans,double *information)
        :TGps(quat_trans),info(information)
    {
        if(info)
        {
            Eigen::Map<Eigen::Matrix<double, 6, 6> > mapInfo(information);
            Eigen::LLT<Eigen::Matrix<double, 6, 6>>  llt(mapInfo);
            sqrt_info = llt.matrixL();
        }

        double quat[4]={TGps[3],TGps[0],TGps[1],TGps[2]};
        ceres::QuaternionToAngleAxis(quat,gpsLog+3);
        for(int i=0;i<3;i++) gpsLog[i]=TGps[i+4];
    }

    template <typename T>
    inline void SE3Mult(const T* Rl,const T* tl,
                        const T* Rr,const T* tr,
                        T* R,T* t)const
    {
        ceres::QuaternionProduct(Rl,Rr,R);
        ceres::UnitQuaternionRotatePoint(Rl,tr,t);
        for(int i=0;i<3;i++)
            t[i]=t[i]+tl[i];
    }

    template <typename T>
    inline void SE3Mult(const T* Rtl,const T* Rtr,T* Rt)const
    {
        SE3Mult(Rtl,Rtl+4,Rtr,Rtr+4,Rt,Rt+4);
    }

    template <typename T> bool
    operator()
    (
            const T* const Rt,    // x,y,z,rx,ry,rz,scale
            T* residuals          // dx,dy,dz,drx,dry,drz error=T12*T2^-1*T1
            )
    const
    {
//        T quat[4];
//        ceres::AngleAxisToQuaternion(Rt+3,quat);
//        for(int i=1;i<4;i++) quat[i]=-quat[i];
//        T transinv[3];
//        ceres::UnitQuaternionRotatePoint(quat,Rt,transinv);
//        for(int i=0;i<3;i++) transinv[i]=-transinv[i];

//        T gps[7]={T(TGps[3]),T(TGps[0]),T(TGps[1]),T(TGps[2]),T(TGps[4]),T(TGps[5]),T(TGps[6])};//qx,qy,qz,qw,x,y,z
//        T error[7];
//        SE3Mult(gps,gps+4,quat,transinv,error,error+4);
//        ceres::QuaternionToAngleAxis(error,residuals+3);
//        for(int i=0;i<3;i++)
//            residuals[i]=error[i+4];

        for(int i=0;i<6;i++) residuals[i]=Rt[i]-gpsLog[i];
        double theta=0;
        for(int i=3;i<6;i++)
        {
            double var=*(double*)(residuals+i);
            theta+=var*var;
        }
        if(theta>M_PI*M_PI)
        {
            theta=sqrt(theta);
            int    numPI=int(theta/M_PI);
            if(numPI|1) numPI++;
            double thetaAfter=theta-numPI*M_PI;
            double factor=thetaAfter/theta;
            for(int i=3;i<6;i++)
            {
                double& var=*(double*)(residuals+i);
                var*=factor;
            }
        }
        if(info)
        {
            T* r=residuals;
            double  res[6]={0,0,0,0,0,0};
            const Eigen::Matrix<double,6,6>& si=sqrt_info;
            for(int i=0;i<num_residuals();i++)
                for(int j=0;j<num_residuals();j++)
                    res[i]+=si(i,j)*(*(double*)&r[j]);
            for(int i=0;i<num_residuals();i++)
                *(double*)(residuals+i)=res[i];
        }
        return true;
    }

    static int num_residuals() { return 6;}
private:
    const double* TGps;//SE3_{12}:=SE3_1^{-1}*SE3_2
    const double* info;
    double gpsLog[6];
    Eigen::Matrix<double,6,6> sqrt_info;
};

class EdgeGPSCostFunction : public ceres::AutoDiffCostFunction<EdgeGPSResiduals,6,7>
{
public:
    EdgeGPSCostFunction(double* quat_trans,double *info):
        ceres::AutoDiffCostFunction<EdgeGPSResiduals,6,7>(new EdgeGPSResiduals(quat_trans,info)){}
};
