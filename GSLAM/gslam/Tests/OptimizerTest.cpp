#include "gtest.h"

#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/Random.h>
#include <GSLAM/core/Glog.h>
#include <GSLAM/core/Timer.h>
#include <GSLAM/core/VecParament.h>

typedef GSLAM::TicToc Tictac;
using namespace std;
using namespace GSLAM;

struct PlyObject
{
    PlyObject(string file2save="out.ply"):_file2save(file2save){}
    ~PlyObject(){save(_file2save);}
    typedef pi::Point3f Vertex3f;
    typedef pi::Point3ub Color3b;

    std::string _file2save;
    std::vector<pi::Point3f>  vertices;
    std::vector<unsigned int> faces;
    std::vector<pi::Point3f>  normals;
    std::vector<pi::Point3ub> colors;
    std::vector<unsigned int> edges;

    void addPoint(Point3d pt,Color3b color=Color3b(255,255,255),pi::Point3f normal=Point3f(0,0,1))
    {
        vertices.push_back(pt);
        colors.push_back(color);
        normals.push_back(normal);
    }

    void addLine(Point3d first,Point3d second,Color3b color=Color3b(255,255,255),pi::Point3f normal=Point3f(0,0,1))
    {
        edges.push_back((uint32_t)vertices.size());
        edges.push_back((uint32_t)vertices.size()+1);
        addPoint(first,color,normal);
        addPoint(second,color,normal);
    }

    bool save(string filename)
    {
        if(filename.substr(filename.find_last_of('.')+1)!="ply") return false;
        std::fstream file;
        file.open(filename.c_str(),std::ios::out|std::ios::binary);
        if(!file.is_open()){
            fprintf(stderr,"\nERROR: Could not open File %s for writing!",(filename).c_str());
            return false;
        }

        uint32_t _verticesPerFace=3;
        bool binary=false;

        file << "ply";
        if(binary)file << "\nformat binary_little_endian 1.0";
        else file << "\nformat ascii 1.0";
        file << "\nelement vertex " << vertices.size();
        file << "\nproperty float32 x\nproperty float32 y\nproperty float32 z";
        if(normals.size())
            file << "\nproperty float32 nx\nproperty float32 ny\nproperty float32 nz";
        if(colors.size())
            file << "\nproperty uchar red\nproperty uchar green\nproperty uchar blue";
        if(faces.size()){
            file << "\nelement face " << faces.size()/_verticesPerFace;
            file << "\nproperty list uint8 int32 vertex_indices";
        }
        if(edges.size()){
            file << "\nelement edge " << edges.size()/2;
            file << "\nproperty int vertex1\nproperty int vertex2";
        }
        file << "\nend_header";
        if(binary) file << "\n";

        for(unsigned int i=0;i<vertices.size();i++){
            if(binary){
                file.write((char*)(&(vertices[i])),sizeof(Vertex3f));
            }
            else file << "\n" << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z;

            if(normals.size())
            {
                if(binary){
                    file.write((char*)(&(normals[i])),sizeof(Vertex3f));
                }
                else file << " " << normals[i].x << " " << normals[i].y << " " << normals[i].z;
            }
            if(colors.size()){
                if(binary){
                    file.write((char*)(&(colors[i])),sizeof(Color3b));
                }
                else file << " " << (int)(colors[i].x) << " " << (int)(colors[i].y) << " " << (int)(colors[i].z);
            }
        }
        for(unsigned int i=0;i<faces.size();i+=_verticesPerFace){
            if(binary){
                file.write((char*)(&_verticesPerFace),sizeof(uchar));
            }
            else file << "\n" << (int)_verticesPerFace;
            for(unsigned int j=0;j<_verticesPerFace;j++)
                if(binary){
                    unsigned int idx = faces[i+j];
                    file.write((char*)(&idx),sizeof(unsigned int));
                }
                else file << " " << (faces[i+j]);
        }
        for(unsigned int i=0;i<edges.size();i+=2){
            if(binary){
                unsigned int idx = edges[i];
                file.write((char*)(&idx),sizeof(unsigned int));
                idx = edges[i+1]; file.write((char*)(&idx),sizeof(unsigned int));
            }
            else file << "\n " << edges[i] << " " << edges[i+1];
        }

        file.close();
        return true;
    }
};

TEST(Optimizer,OptimizePoseSimulate)
{
    for(int It=0;It<1;It++)
    {
        // Prepare simulation data
        int N=svar.GetInt("SimulateNumber",200);//number of matches
        std::vector<std::pair<pi::Point3d, pi::Point3d> > groundMatches,estMatches;
        std::vector<pi::Point2d>                          groundIdepth,estIDepth;
        GSLAM::SE3                                        groundPose,estPose;
        groundPose=GSLAM::SE3(pi::SO3d::exp(Point3d(0,0,1)),pi::Point3d(1,0,0));
        for(int i=0;i<N;i++)
        {
            pi::Point3d p1(GSLAM::Random::RandomValue(-1.,1.),GSLAM::Random::RandomValue(-1.,1.),1);
            pi::Point2d idepthSigma(GSLAM::Random::RandomGaussianValue(0.1,0.1),i<(N/2)?0.1:0);
            if(idepthSigma.x<=0) {idepthSigma.x=0.1;}
            pi::Point3d p2(groundPose.inverse()*(p1*(1./idepthSigma.x)));
            if(p2.z) p2=p2*(1./p2.z);
            groundMatches.push_back(std::make_pair(p1,p2));
            groundIdepth.push_back(idepthSigma);
            // add noise
            if(i<(N/2))
            {
                double idepthNoise=GSLAM::Random::RandomGaussianValue(0.1,0.05);
                pi::Point3d projectNoise(GSLAM::Random::RandomGaussianValue(0.,i<(N/3)?0.1:0.01),
                                         GSLAM::Random::RandomGaussianValue(0.,i<(N/3)?0.1:0.01),0);
                estMatches.push_back(std::make_pair(p1,p2+projectNoise));
                estIDepth.push_back(idepthSigma+pi::Point2d(idepthNoise,0.1));
            }
            else
            {
                pi::Point3d projectNoise(GSLAM::Random::RandomGaussianValue(0.,0.01),
                                         GSLAM::Random::RandomGaussianValue(0.,0.01),0);
                estMatches.push_back(std::make_pair(p1,p2+projectNoise));
                estIDepth.push_back(idepthSigma);
            }
        }

        SPtr<Optimizer> opt=Optimizer::create();
        if(!opt)
        {
            LOG(ERROR)<<"No valid optimizer plugin!";
            return;
        }
        opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
        EXPECT_TRUE(opt->optimizePose(estMatches,estIDepth,estPose));

        LOG_IF(INFO,opt->_config.verbose)<<"GroundPose:"<<groundPose<<",EstPose:"<<estPose<<std::endl;
    }
}

TEST(Optimizer,OptimizePnPSimulate)
{
    // Prepare simulation data
    int N=svar.GetInt("SimulateNumber",2000);//number of matches
    std::vector<std::pair<pi::Point3d, pi::Point3d> > groundMatches,estMatches;
    GSLAM::SE3                                        groundPose,estPose;
    pi::Array_<double,6> array;
    for(int i=0;i<array.size();i++)
        array.data[i]=GSLAM::Random::RandomValue<double>();
    groundPose=GSLAM::SE3::exp(array);
    estPose=groundPose*GSLAM::SE3(pi::SO3d::exp(Point3d(0.1,0.1,0.1)),pi::Point3d(1,2,3));
    for(int i=0;i<N;i++)
    {
        pi::Point3d pCamera(GSLAM::Random::RandomValue(-1.,1.),GSLAM::Random::RandomValue(-1.,1.),1);
        pi::Point3d pWorld(groundPose*(pCamera*GSLAM::Random::RandomGaussianValue(10.,1.)));

        groundMatches.push_back(std::make_pair(pWorld,pCamera));
        // add noise
        if(true)
        {
            pi::Point3d projectNoise(GSLAM::Random::RandomGaussianValue(0.,i<(0.5*N)?0.1:0.01),
                                     GSLAM::Random::RandomGaussianValue(0.,i<(0.5*N)?0.1:0.01),0);
            pCamera=pCamera+projectNoise;
            pCamera=pCamera*4;
        }
        estMatches.push_back(std::make_pair(pWorld,pCamera));
    }


    SPtr<Optimizer> opt=Optimizer::create();
    if(!opt)
    {
        LOG(ERROR)<<"No valid optimizer plugin!";
        return;
    }
    opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
    GSLAM::TicToc ticToc;
    EXPECT_TRUE(opt->optimizePnP(estMatches,estPose));

    LOG_IF(INFO,opt->_config.verbose)<<"GroundPose:"<<groundPose<<",EstPose:"<<estPose<<",Time:"<<ticToc.Tac()*1000<<"ms"<<std::endl;
}

TEST(Optimizer,ICPSimulate)
{
    int N=svar.GetInt("SimulateNumber",2000);
    std::vector<Point3d> srcPoints;
    for(int i=0;i<N;i++) srcPoints.push_back(Point3d(GSLAM::Random::RandomValue(0.,1.),
                                                     GSLAM::Random::RandomValue(0.,1.),
                                                     GSLAM::Random::RandomValue(0.,1.)));

    pi::Array_<double,6> array;
    for(int i=0;i<array.size();i++)
        array.data[i]=GSLAM::Random::RandomValue<double>();
    GSLAM::SIM3 sim3(GSLAM::SE3::exp(array),2.);
//    sim3.get_translation()=Point3d(0,0,0);
    sim3.get_rotation()=GSLAM::SO3::exp(Point3d(0.3,0.3,0.3));

    std::vector<std::pair<Point3d,Point3d> > matches;
    for(int i=0;i<N;i++)
    {
        Point3d noise(Random::RandomGaussianValue(0.,0.001),
                      Random::RandomGaussianValue(0.,0.001),
                      Random::RandomGaussianValue(0.,0.001));
        matches.push_back(std::make_pair(sim3*srcPoints[i],srcPoints[i]+noise));
    }

    GSLAM::SIM3 estSim3;
    SPtr<Optimizer> opt=Optimizer::create();
    if(!opt)
    {
        LOG(ERROR)<<"No valid optimizer plugin!";
        return;
    }
    opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
    EXPECT_TRUE(opt->optimizeICP(matches,estSim3,UPDATE_KF_SIM3));

    PlyObject ply("ICPSimulate.ply");
    for(std::pair<Point3d,Point3d>& mt:matches)
    {
        ply.addPoint(mt.second,ColorType(255,0,0));
        ply.addPoint(mt.first,ColorType(0,255,0));
        ply.addPoint(estSim3*mt.second,ColorType(0,0,255));
    }

    DLOG(INFO)<<"GroudSim3:"<<sim3.get_se3()<<","<<sim3.get_scale()
             <<",Estimation:"<<estSim3.get_se3()<<","<<estSim3.get_scale();
}

TEST(Optimizer,SE3GraphSimulate)
{
     int N=svar.GetInt("SE3GraphSimulate.Number",1000);
     double noise=svar.GetDouble("SE3GraphSimulate.Noise",1e-3);
     GSLAM::BundleGraph graph;
     GSLAM::KeyFrameEstimzation currentFrame={GSLAM::SE3(GSLAM::SO3(),Point3d(1,0,0)),UPDATE_KF_SE3};
     GSLAM::SE3 odometer;
     double angle=M_PI*2/N;
     odometer.get_translation()=Point3d(cos(angle)-1.,sin(angle),0);
     odometer.get_rotation()=SO3::exp(Point3d(0,0,-angle));
     double information[36];
     for(int i=0;i<6;i++)
         for(int j=0;j<6;j++)
         {
             if(i==j) information[i*6+j]=1;
             else information[i*6+j]=0;
         }

     for(size_t i=0;i<N;i++)
     {
         GSLAM::SE3 noised_odo=odometer*GSLAM::SE3(SO3::exp(Point3d(Random::RandomGaussianValue(0.,noise),
                                                                    Random::RandomGaussianValue(0.,noise),
                                                                    Random::RandomGaussianValue(0.,noise))),
                                                   Point3d(Random::RandomGaussianValue(0.,noise),
                                                           Random::RandomGaussianValue(0.,noise),
                                                           Random::RandomGaussianValue(0.,noise)));
         graph.keyframes.push_back(currentFrame);
         graph.se3Graph.push_back({i,i+1,noised_odo,information});
         if(i==0)
             graph.keyframes.back().dof=UPDATE_KF_NONE;// fixed
         else if(i+1==N)
             graph.se3Graph.back().secondId=0;//loop

         // move on
         currentFrame.estimation=currentFrame.estimation*noised_odo;

     }
     PlyObject ply("SE3GraphSimulate.ply");
     for(KeyFrameEstimzation& kf:graph.keyframes) ply.addPoint(kf.estimation.get_translation(),GSLAM::ColorType(255,0,0));
     SPtr<Optimizer> opt=Optimizer::create();
     if(!opt)
     {
         LOG(ERROR)<<"No valid optimizer plugin!";
         return;
     }
     opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
     EXPECT_TRUE(opt->optimize(graph));

     for(KeyFrameEstimzation& kf:graph.keyframes) ply.addPoint(kf.estimation.get_translation(),GSLAM::ColorType(0,255,0));
     for(SE3Edge& edge:graph.se3Graph)
         ply.addLine(graph.keyframes[edge.firstId].estimation.get_translation(),
             graph.keyframes[edge.firstId].estimation.get_translation(),GSLAM::ColorType(0,0,255));
}

TEST(Optimizer,SIM3GraphSimulate)
{
    int N=svar.GetInt("SIM3GraphSimulate.Number",1000);
    double noise=svar.GetDouble("SIM3GraphSimulate.Noise",1e-3);
    GSLAM::BundleGraph graph;
    GSLAM::KeyFrameEstimzation currentFrame={GSLAM::SE3(GSLAM::SO3(),Point3d(1,0,0)),UPDATE_KF_SIM3};
    GSLAM::SIM3 odometer;
    double angle=M_PI*2/N;
    odometer.get_translation()=Point3d(cos(angle)-1.,sin(angle),0);
    odometer.get_rotation()=SO3::exp(Point3d(0,0,-angle));

    double information[49];
    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
        {
            if(i==j) information[i*7+j]=1;
            else information[i*7+j]=0;
        }

    for(size_t i=0;i<N;i++)
    {
        GSLAM::SIM3 noised_odo=odometer*GSLAM::SIM3(SO3::exp(Point3d(Random::RandomGaussianValue(0.,noise),
                                                                   Random::RandomGaussianValue(0.,noise),
                                                                   Random::RandomGaussianValue(0.,noise))),
                                                  Point3d(Random::RandomGaussianValue(0.,noise),
                                                          Random::RandomGaussianValue(0.,noise),
                                                          Random::RandomGaussianValue(0.,noise)),
                                                   Random::RandomGaussianValue(1.,noise));
        graph.keyframes.push_back(currentFrame);
        graph.sim3Graph.push_back({i,i+1,noised_odo,information});
        if(i==0)
            graph.keyframes.back().dof=UPDATE_KF_NONE;// fixed
        else if(i+1==N)
            graph.sim3Graph.back().secondId=0;//loop

        // move on
        currentFrame.estimation=currentFrame.estimation*noised_odo;

    }
    PlyObject ply("SIM3GraphSimulate.ply");
    for(KeyFrameEstimzation& kf:graph.keyframes) ply.addPoint(kf.estimation.get_translation(),GSLAM::ColorType(255,0,0));
    SPtr<Optimizer> opt=Optimizer::create();
    if(!opt)
    {
        LOG(ERROR)<<"No valid optimizer plugin!";
        return;
    }
    opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
    EXPECT_TRUE(opt->optimize(graph));

    for(KeyFrameEstimzation& kf:graph.keyframes) ply.addPoint(kf.estimation.get_translation(),GSLAM::ColorType(0,255,0));
    for(SIM3Edge& edge:graph.sim3Graph)
        ply.addLine(graph.keyframes[edge.firstId].estimation.get_translation(),
            graph.keyframes[edge.firstId].estimation.get_translation(),GSLAM::ColorType(0,0,255));
}

TEST(Optimizer,GPSGraphSimulate)
{
    int N=svar.GetInt("GPSGraphSimulate.Number",1000);
    double noise=svar.GetDouble("GPSGraphSimulate.OdoNoise",1e-3);
    double gpsNoise=svar.GetDouble("GPSGraphSimulate.GPSNoise",2e-3);
    bool withOdometer=svar.GetInt("GPSGraphSimulate.EnableOdometer",1);
    double radius=svar.GetInt("GPSGraphSimulate.Radius",100);
    int    gpsStep=svar.GetInt("GPSGraphSimulate.GPSStep",50);
    VecParament<double> gps_info(std::vector<double>({1,1,0.5,0.1,0.1,0.1}));
    gps_info=svar.get_var("GPSGraphSimulate.GPSInfo",gps_info);
    GSLAM::BundleGraph graph;
    GSLAM::KeyFrameEstimzation currentFrame={GSLAM::SIM3(GSLAM::SO3(),Point3d(radius,0,0),radius),UPDATE_KF_SIM3};
    GSLAM::SIM3 odometer,curGPS=currentFrame.estimation;

    double angle=M_PI*2/N;
    odometer.get_translation()=Point3d(cos(angle)-1.,sin(angle),0);
    odometer.get_rotation()=SO3::exp(Point3d(0,0,-angle));

    double information[49];
    for(int i=0;i<7;i++)
        for(int j=0;j<7;j++)
        {
            if(i==j) information[i*7+j]=1;
            else information[i*7+j]=0;
        }

    double gpsInfo[36];// rotation are not so precise and we only use the translation constraint
    for(int i=0;i<36;i++) gpsInfo[i]=0;
    for(int i=0;i<6;i++) gpsInfo[i*7]=gps_info[i];


    for(size_t i=0;i<N;i++)
    {
        GSLAM::SIM3 noised_odo=odometer*GSLAM::SIM3(SO3::exp(Point3d(Random::RandomGaussianValue(0.,noise),
                                                                   Random::RandomGaussianValue(0.,noise),
                                                                   Random::RandomGaussianValue(0.,noise))),
                                                  Point3d(Random::RandomGaussianValue(0.,noise),
                                                          Random::RandomGaussianValue(0.,noise),
                                                          Random::RandomGaussianValue(0.,noise)),
                                                   Random::RandomGaussianValue(1.,noise));
        GSLAM::SE3 gps_noise=GSLAM::SE3(SO3::exp(Point3d(Random::RandomGaussianValue(0.,gpsNoise),
                                                         Random::RandomGaussianValue(0.,gpsNoise),
                                                         Random::RandomGaussianValue(0.,gpsNoise))),
                                        Point3d(Random::RandomGaussianValue(0.,gpsNoise),
                                                Random::RandomGaussianValue(0.,gpsNoise),
                                                Random::RandomGaussianValue(0.,gpsNoise)));
        GSLAM::SE3 noised_gps=curGPS.get_se3()*gps_noise;
        graph.keyframes.push_back(currentFrame);
        if(i%gpsStep==0)
            graph.gpsGraph.push_back({i,noised_gps,gpsInfo});

        if(withOdometer)
        {
            graph.sim3Graph.push_back({i,i+1,noised_odo,information});
            if(i+1==N)
                graph.sim3Graph.back().secondId=0;//loop
        }

        // move on
        currentFrame.estimation=currentFrame.estimation*noised_odo;
        curGPS=curGPS*odometer;
    }
    PlyObject ply("GPSGraphSimulate.ply");
    for(KeyFrameEstimzation& kf:graph.keyframes) ply.addPoint(kf.estimation.get_translation(),GSLAM::ColorType(255,0,0));
    SPtr<Optimizer> opt=Optimizer::create();
    if(!opt)
    {
        LOG(ERROR)<<"No valid optimizer plugin!";
        return;
    }
    opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
    EXPECT_TRUE(opt->optimize(graph));

    for(KeyFrameEstimzation& kf:graph.keyframes) ply.addPoint(kf.estimation.get_translation(),GSLAM::ColorType(0,255,0));
    for(GPSEdge& edge:graph.gpsGraph)
        ply.addLine(graph.keyframes[edge.frameId].estimation.get_translation(),
            edge.measurement.get_translation(),GSLAM::ColorType(0,0,255));
}

TEST(Optimizer,MapPointBundleSimulate)
{
    // Prepare simulation data
    PlyObject ply("MapPointBundleSimulate.ply");// ground in green , initial in red, final in blue
    int N=svar.GetInt("MapPointBundleSimulate.Number",2000);//number of matches
    double noise=svar.GetDouble("MapPointBundleSimulate.Noise",1e-2);
    double projNoise=svar.GetDouble("MapPointBundleSimulate.ProjectNoise",1e-5);

    pi::Array_<double,6> array;
    for(int i=0;i<array.size();i++)
        array.data[i]=GSLAM::Random::RandomValue<double>();

    GSLAM::SE3 firstPose =GSLAM::SE3::exp(array);
    GSLAM::SE3 secondPose=firstPose*GSLAM::SE3(SO3::exp(Point3d(0.1,0.1,0.1)),Point3d(1.,1.,1.));
    GSLAM::SE3 noisedSecondPose=secondPose*GSLAM::SE3(SO3::exp(Point3d(Random::RandomGaussianValue(0.,noise),
                                                                       Random::RandomGaussianValue(0.,noise),
                                                                       Random::RandomGaussianValue(0.,noise))),
                                                      Point3d(Random::RandomGaussianValue(0.,noise),
                                                              Random::RandomGaussianValue(0.,noise),
                                                              Random::RandomGaussianValue(0.,noise)));

    BundleGraph graph;
    graph.keyframes.push_back({firstPose,UPDATE_KF_NONE});
    graph.keyframes.push_back({noisedSecondPose,UPDATE_KF_SE3});
    ply.addPoint(firstPose.get_translation(),ColorType(0,255,0));
    ply.addPoint(secondPose.get_translation(),ColorType(0,255,0));
    ply.addPoint(noisedSecondPose.get_translation(),ColorType(255,0,0));
    for(int i=0;i<N;i++)
    {
        Point3d pCamera(GSLAM::Random::RandomValue(-1.,1.),GSLAM::Random::RandomValue(-1.,1.),1);
        Point3d pWorld(firstPose*(pCamera*GSLAM::Random::RandomGaussianValue(10.,0.)));
        Point3d pCamera2=secondPose.inverse()*pWorld;
        if(pCamera2.z<=1) continue;    // should in front of camera
        pCamera2=pCamera2/pCamera2.z;
        if(fabs(pCamera2.x)>1||fabs(pCamera2.y)>1) continue;// should cam be seen by camera
        Point3d pWorldNoise(Random::RandomGaussianValue(0.,noise),
                            Random::RandomGaussianValue(0.,noise),
                            Random::RandomGaussianValue(0.,noise));

        Point3d pCameraNoise0(Random::RandomGaussianValue(0.,projNoise),
                              Random::RandomGaussianValue(0.,projNoise),0);
        Point3d pCameraNoise1(Random::RandomGaussianValue(0.,projNoise),
                              Random::RandomGaussianValue(0.,projNoise),0);

        graph.mappoints.push_back({pWorld+pWorldNoise,true});
        graph.mappointObserves.push_back({graph.mappoints.size()-1,0,pCamera+pCameraNoise0});
        graph.mappointObserves.push_back({graph.mappoints.size()-1,1,pCamera2+pCameraNoise1});

        ply.addPoint(pWorld,ColorType(0,255,0));
        ply.addPoint(pWorld+pWorldNoise,ColorType(255,0,0));
    }

    SPtr<Optimizer> opt=Optimizer::create();
    if(!opt)
    {
        LOG(ERROR)<<"No valid optimizer plugin!";
        return;
    }
    opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
    EXPECT_TRUE(opt->optimize(graph));

    for(MapPointEstimation& est:graph.mappoints)
        ply.addPoint(est.first,ColorType(0,0,255));
    ply.addPoint(graph.keyframes[1].estimation.get_translation(),ColorType(0,0,255));
}

TEST(Optimizer,InvDepthBundleSimulate)
{
    // Prepare simulation data
    PlyObject ply("InvDepthBundleSimulate.ply");// ground in green , initial in red, final in blue
    int N=svar.GetInt("InvDepthBundleSimulate.Number",2000);//number of matches
    double noise=svar.GetDouble("InvDepthBundleSimulate.Noise",1e-2);
    double projNoise=svar.GetDouble("InvDepthBundleSimulate.ProjectNoise",1e-5);

    pi::Array_<double,6> array;
    for(int i=0;i<array.size();i++)
        array.data[i]=GSLAM::Random::RandomValue<double>();

    GSLAM::SE3 firstPose =GSLAM::SE3::exp(array);
    GSLAM::SE3 secondPose=firstPose*GSLAM::SE3(SO3::exp(Point3d(0.1,0.1,0.1)),Point3d(1.,1.,1.));
    GSLAM::SE3 noisedSecondPose=secondPose
            *GSLAM::SE3(SO3::exp(Point3d(Random::RandomGaussianValue(0.,noise),
                                         Random::RandomGaussianValue(0.,noise),
                                         Random::RandomGaussianValue(0.,noise))),
                        Point3d(Random::RandomGaussianValue(0.,noise),
                                Random::RandomGaussianValue(0.,noise),
                                Random::RandomGaussianValue(0.,noise)));

    BundleGraph graph;
    graph.keyframes.push_back({firstPose,UPDATE_KF_NONE});
    graph.keyframes.push_back({noisedSecondPose,UPDATE_KF_SE3});
    ply.addPoint(firstPose.get_translation(),ColorType(0,255,0));
    ply.addPoint(secondPose.get_translation(),ColorType(0,255,0));
    ply.addPoint(noisedSecondPose.get_translation(),ColorType(255,0,0));
    for(int i=0;i<N;i++)
    {
        Point3d pCamera(GSLAM::Random::RandomValue(-1.,1.),GSLAM::Random::RandomValue(-1.,1.),1);
        Point2d pDepth(GSLAM::Random::RandomGaussianValue(10.,0.),0);
        Point3d pWorld(firstPose*(pCamera*pDepth.x));
        Point3d pCamera2=secondPose.inverse()*pWorld;
        if(pCamera2.z<=1) continue;    // should in front of camera
        pCamera2=pCamera2/pCamera2.z;
        if(fabs(pCamera2.x)>1||fabs(pCamera2.y)>1) continue;// should cam be seen by camera
        double depthNoise(Random::RandomGaussianValue(0.,noise));

        Point3d pCameraNoise0(Random::RandomGaussianValue(0.,projNoise),
                              Random::RandomGaussianValue(0.,projNoise),0);
        Point3d pCameraNoise1(Random::RandomGaussianValue(0.,projNoise),
                              Random::RandomGaussianValue(0.,projNoise),0);

        graph.invDepths.push_back({0,pCamera,Point2d(1./(pDepth.x+depthNoise),100),UPDATE_ID_IDEPTH});
        graph.invDepthObserves.push_back({graph.invDepths.size()-1,1,pCamera2+pCameraNoise1});

        ply.addPoint(pWorld,ColorType(0,255,0));
        ply.addPoint(firstPose*(pCamera*(pDepth.x+depthNoise)),ColorType(255,0,0));
    }

    SPtr<Optimizer> opt=Optimizer::create();
    if(!opt)
    {
        LOG(ERROR)<<"No valid optimizer plugin!";
        return;
    }
    opt->_config.verbose=svar.GetInt("Optimizer.Verbose",0);
    EXPECT_TRUE(opt->optimize(graph));

    for(InvDepthEstimation& est:graph.invDepths)
    {
        ply.addPoint(graph.keyframes[est.frameId].estimation*(est.anchor/est.estimation.x),ColorType(0,0,255));
    }
    ply.addPoint(graph.keyframes[1].estimation.get_translation(),ColorType(0,0,255));
}

