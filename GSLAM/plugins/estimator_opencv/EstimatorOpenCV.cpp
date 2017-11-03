#include "../../core/Estimator.h"
#include <opencv2/opencv.hpp>

namespace GSLAM
{
class EstimatorOpenCV : public Estimator
{
public:
    EstimatorOpenCV()
    {}

    inline std::vector<cv::Point2d> toInputArray(const std::vector<Point2d>& input)const
    {
        return *(std::vector<cv::Point2d>*)&input;
    }

    inline std::vector<cv::Point3d> toInputArray(const std::vector<Point3d>& input)const
    {
        return *(std::vector<cv::Point3d>*)&input;
    }

    inline bool toReturn(double* Ret,cv::Mat result)const
    {
        if(result.empty()) return false;
        result.convertTo(result,CV_64F);
        memcpy(Ret,result.data,sizeof(double)*result.total());
        return true;
    }

    // 2D corrospondences
    bool findHomography(double* H,//3x3 dof=8
                        const std::vector<Point2d>& srcPoints,
                        const std::vector<Point2d>& dstPoints,
                        int method, double ransacReprojThreshold,
                        std::vector<uchar>* mask=NULL)const{
        if(!H) return false;
        return toReturn(H,cv::findHomography(toInputArray(srcPoints),toInputArray(dstPoints),
                                             method,ransacReprojThreshold,mask?(*mask):cv::noArray()));
    }

    bool findAffine2D(double* A,//2X3
                      const std::vector<Point2d>& srcPoints,
                      const std::vector<Point2d>& dstPoints,
                      bool fullAffine)const{
        return toReturn(A,cv::estimateRigidTransform(toInputArray(srcPoints),toInputArray(dstPoints),fullAffine));
    }

    bool findFundamental(double* F,//3x3
                         const std::vector<Point2d>& points1,
                         const std::vector<Point2d>& points2,
                         int method=0, double param1=3., double param2=0.99,
                         std::vector<uchar>* mask=NULL)const
    {
        return toReturn(F,cv::findFundamentalMat(toInputArray(points1),toInputArray(points2),
                               method,param1,param2,mask?(*mask):cv::noArray()));
    }

    bool findEssentialMatrix(double* E,//3x3 dof=5
                             const std::vector<Point2d>& points1,
                             const std::vector<Point2d>& points2,
                             int method=0, double param1=0.01, double param2=0.99,
                             std::vector<uchar>* mask=NULL)const { return false;  }

    // 3D corrospondences
    bool findSIM3(SIM3& S,
                  const std::vector<Point3d>& from,
                  const std::vector<Point3d>& to,
                  int method=0, double ransacThreshold=-1,
                  std::vector<uchar>* mask=NULL) const{
        // Custom implementation of:
        // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

        /// 1. Compute the centre of two point set and translate points to centre

        pi::Point3d centre_Track(0,0,0);
        pi::Point3d centre_GPS(0,0,0);
        size_t Num=from.size();
        if(to.size()!=Num) return -1;

        for(size_t i=0;i<Num;i++)
        {
            centre_Track=centre_Track+from[i];
            centre_GPS  =centre_GPS+to[i];
        }
        centre_Track=centre_Track/(double)Num;
        centre_GPS=centre_GPS/(double)Num;

        cv::Mat Pr1(3,Num,CV_64F); // Relative coordinates to centroid (set 1)
        cv::Mat Pr2(3,Num,CV_64F); // Relative coordinates to centroid (set 2)
        for(size_t i=0;i<Num;i++)
        {
            pi::Point3d pt1=from[i];
            pi::Point3d pt2=to[i];
            pt1=pt1-centre_Track;
            pt2=pt2-centre_GPS;
            cv::Mat(3,1,Pr1.type(),&pt1).copyTo(Pr1.col(i));
            cv::Mat(3,1,Pr1.type(),&pt2).copyTo(Pr2.col(i));
        }

        /// 2. Compute M ,N matrix

        cv::Mat M = Pr2*Pr1.t();

        double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        cv::Mat N(4,4,Pr1.type());

        N11 = M.at<double>(0,0)+M.at<double>(1,1)+M.at<double>(2,2);
        N12 = M.at<double>(1,2)-M.at<double>(2,1);
        N13 = M.at<double>(2,0)-M.at<double>(0,2);
        N14 = M.at<double>(0,1)-M.at<double>(1,0);
        N22 = M.at<double>(0,0)-M.at<double>(1,1)-M.at<double>(2,2);
        N23 = M.at<double>(0,1)+M.at<double>(1,0);
        N24 = M.at<double>(2,0)+M.at<double>(0,2);
        N33 = -M.at<double>(0,0)+M.at<double>(1,1)-M.at<double>(2,2);
        N34 = M.at<double>(1,2)+M.at<double>(2,1);
        N44 = -M.at<double>(0,0)-M.at<double>(1,1)+M.at<double>(2,2);

        N = (cv::Mat_<double>(4,4) << N11, N12, N13, N14,
                                     N12, N22, N23, N24,
                                     N13, N23, N33, N34,
                                     N14, N24, N34, N44);


        /// 3. Get rotation from eigenvector of the highest eigenvalue

        cv::Mat eval, evec;

        cv::eigen(N,eval,evec);
        pi::SO3d so3(evec.at<double>(0,1),
                     evec.at<double>(0,2),
                     evec.at<double>(0,3),
                     evec.at<double>(0,0));

        cv::Mat mR12i(3,3,CV_64F);
        pi::Point3d vec_p=so3.inv().ln();
        cv::Mat vec(1,3,CV_64F,&vec_p);
        cv::Rodrigues(vec,mR12i);

        /// 4: Rotate set 2 and compute scale

        cv::Mat P3 = mR12i*Pr2;

        double nom = Pr1.dot(P3);
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        cv::pow(P3,2,aux_P3);
        double den = 0;

        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<double>(i,j);
            }
        }

        double scale = den/nom;

        /// 5. Compute translation and get SIM3


        pi::Point3d translation = centre_GPS - (so3*centre_Track)*scale;

        S=pi::SIM3d(so3,translation,scale);
        return true;
    }

    bool findAffine3D(double* A,
                      const std::vector<Point3d>& src,
                      const std::vector<Point3d>& dst,
                      std::vector<int>* inliers = NULL,
                      double ransacThreshold=3, double confidence=0.99) const{
        cv::Mat Acv;
        if(cv::estimateAffine3D(toInputArray(src),toInputArray(dst),
                                    Acv,inliers?(*inliers):cv::noArray(),ransacThreshold,confidence)!=0) return false;
        return toReturn(A,Acv);
    }

    bool findPlane(SE3& plane,
                   const std::vector<Point3d>& points,
                   int method=0, double thresholdZ=-1.,
                   std::vector<uchar>* mask=NULL) const {
        using namespace std;
        unsigned int nPoints = points.size();
        if(nPoints < 3)
        {
            LOG(ERROR) << "CalcPlane: too few points to calc plane.";
            return false;
        }
        int nRansacs =std::max(100.,points.size()*0.2);
        pi::Point3d v3BestMean;
        pi::Point3d v3BestNormal;
        double dBestDistSquared = 9999999999999999.9;

        for(int i=0; i<nRansacs; i++)
        {
            int nA = rand()%nPoints;
            int nB = nA;
            int nC = nA;
            while(nB == nA)
                nB = rand()%nPoints;
            while(nC == nA || nC==nB)
                nC = rand()%nPoints;

            pi::Point3d v3Mean = 0.33333333 * (points[nA] +
                                               points[nB] +
                                               points[nC]);

            pi::Point3d v3CA = points[nC]  - points[nA];
            pi::Point3d v3BA = points[nB]  - points[nA];
            pi::Point3d v3Normal = v3CA ^ v3BA;
            if(v3Normal * v3Normal  == 0)
                continue;
            v3Normal=v3Normal.normalize();

            double dSumError = 0.0;
            for(unsigned int i=0; i<nPoints; i++)
            {
                pi::Point3d v3Diff = points[i] - v3Mean;
                double dDistSq = v3Diff * v3Diff;
                if(dDistSq == 0.0)
                    continue;
                double dNormDist = fabs(v3Diff * v3Normal);

                if(dNormDist > thresholdZ)
                    dNormDist = thresholdZ;
                dSumError += dNormDist;
            }
            if(dSumError < dBestDistSquared)
            {
                dBestDistSquared = dSumError;
                v3BestMean = v3Mean;
                v3BestNormal = v3Normal;
            }
        }

        // Done the ransacs, now collect the supposed inlier set
        vector<pi::Point3d > vv3Inliers;
        std::vector<int> outliers;
        outliers.clear();
        outliers.reserve(nPoints);
        vv3Inliers.reserve(nPoints);

        for(unsigned int i=0; i<nPoints; i++)
        {
            pi::Point3d v3Diff = points[i] - v3BestMean;
            double dDistSq = v3Diff * v3Diff;
            if(dDistSq == 0.0)
                continue;
            double dNormDist = fabs(v3Diff * v3BestNormal);
            if(dNormDist < thresholdZ)
            {
                vv3Inliers.push_back(points[i]);
            }
            else
                outliers.push_back(i);
        }

        // With these inliers, calculate mean and cov
        pi::Point3d v3MeanOfInliers(0,0,0);
        for(unsigned int i=0; i<vv3Inliers.size(); i++)
            v3MeanOfInliers=v3MeanOfInliers+vv3Inliers[i];
        v3MeanOfInliers =v3MeanOfInliers*(1.0 / vv3Inliers.size());

        cv::Mat A=cv::Mat::zeros(3,3,CV_64F);
        double* a=(double*)A.data;
        for(unsigned int i=0; i<vv3Inliers.size(); i++)
        {
            pi::Point3d d = vv3Inliers[i] - v3MeanOfInliers;
            a[0]+=d.x*d.x; a[1]+=d.x*d.y; a[2]+=d.x*d.z;
            a[3]+=d.y*d.x; a[4]+=d.y*d.y; a[5]+=d.y*d.z;
            a[6]+=d.z*d.x; a[7]+=d.z*d.y; a[8]+=d.z*d.z;
        };
        cv::Mat eValuesMat;
        cv::Mat eVectorsMat;
        cv::eigen(A,eValuesMat,eVectorsMat);
    //    cout<<"Eigen Values:"<<eValuesMat<<"\nVecs:"<<eVectorsMat<<endl;
        int minIdx=0;
        if(eValuesMat.at<double>(minIdx)>eValuesMat.at<double>(1)) minIdx=1;
        if(eValuesMat.at<double>(minIdx)>eValuesMat.at<double>(2)) minIdx=2;

        v3BestNormal=pi::Point3d(eVectorsMat.at<double>(minIdx,0),
                                 eVectorsMat.at<double>(minIdx,1),
                                 eVectorsMat.at<double>(minIdx,2));

        pi::Point3d vx,vy,vz;
        if(v3BestNormal.z<0)
        vz=-v3BestNormal;
        else vz=v3BestNormal;
        vx=(vz^pi::Point3d(0,-1,0)).normalize();
        vy=(vz^vx);
        double r[9];
        r[0]=vx.x;r[1]=vy.x;r[2]=vz.x;
        r[3]=vx.y;r[4]=vy.y;r[5]=vz.y;
        r[6]=vx.z;r[7]=vy.z;r[8]=vz.z;
        SE3 se3Aligner;
        se3Aligner.get_translation() = v3MeanOfInliers;
        se3Aligner.get_rotation().fromMatrix(r);
        plane=se3Aligner;

        if(mask)
        {
            mask->resize(points.size(),1);
            for(int i:outliers)  (*mask)[i]=0;
        }
        return true;
    }

    // 2D&3D corrospondences
    bool findPnPRansac(  SE3& world2camera,
                         const std::vector<Point3d>& objectPoints,
                         const std::vector<Point2d>& imagePoints,
                         const GSLAM::Camera&        camera,
                         bool useExtrinsicGuess = false,
                         int iterationsCount = 100,
                         float reprojectionError = 8.0,
                         int minInliersCount = 100,
                         std::vector<int>* inliers = NULL,
                         int flags=ITERATIVE)const{
        std::vector<cv::Point2f> planePoints;
        std::vector<cv::Point3f> objectPointsCV;
        planePoints.reserve(imagePoints.size());
        objectPointsCV.reserve(objectPoints.size());

        for(const auto& pt:imagePoints) planePoints.push_back(cv::Point2f(pt.x,pt.y));
        for(const auto& pt:objectPoints) objectPointsCV.push_back(cv::Point3f(pt.x,pt.y,pt.z));

        if(camera.isValid())
        {
            reprojectionError/=camera.getParameters()[2];
            for(cv::Point2f& pt:planePoints)
            {
                Point3d p2d=camera.UnProject(pt.x,pt.y);
                pt.x=p2d.x;pt.y=p2d.y;
            }
        }

        cv::Mat R,t;
        cv::Mat             k=cv::Mat::eye(3,3,CV_32F),distCoeff=cv::Mat::zeros(5,1,CV_32F);
        cv::solvePnPRansac(objectPointsCV,planePoints,k,
                           distCoeff,R,t,useExtrinsicGuess,iterationsCount,reprojectionError,
                           minInliersCount,inliers?(*inliers):cv::noArray(),flags);
        world2camera=pi::SE3d(pi::SO3d::exp(pi::Point3d(R.at<double>(0),R.at<double>(1),R.at<double>(2))),
                      pi::Point3d(t.at<double>(0),t.at<double>(1),t.at<double>(2)));

        return true;
    }

};
}

USE_ESTIMATOR_PLUGIN(GSLAM::EstimatorOpenCV);
