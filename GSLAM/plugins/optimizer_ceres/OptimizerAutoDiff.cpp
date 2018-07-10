#define GSLAM_MINIGLOG_GLOG_LOGGING_H_
#undef __STRICT_ANSI__
#include <math.h>
#include "AutoDiffFactor.hpp"
#include "../../core/Optimizer.h"

using namespace GSLAM;

class OptimizerAutoDiffCeres: public Optimizer
{
public:
    OptimizerAutoDiffCeres(OptimzeConfig config=OptimzeConfig()):Optimizer(config){}

    virtual ~OptimizerAutoDiffCeres(){}

    // Update relative pose agaist the first frame with known or unknown depth
    virtual bool optimizePose(std::vector<std::pair<CameraAnchor,CameraAnchor> >& matches,
                              std::vector<IdepthEstimation>& firstIDepth,GSLAM::SE3&    relativePose,
                              KeyFrameEstimzationDOF dof=UPDATE_KF_SE3,double* information=NULL);

    // Update pose with 3D-2D corrospondences
    virtual bool optimizePnP(const std::vector<std::pair<GSLAM::Point3d,CameraAnchor> >& matches,
                             GSLAM::SE3& pose,KeyFrameEstimzationDOF dof=UPDATE_KF_SE3,double* information=NULL);

    // Update pose with 3D-3D corrospondences
    virtual bool optimizeICP(const std::vector<std::pair<GSLAM::Point3d,GSLAM::Point3d> >& matches,// T_{12}
                             GSLAM::SIM3& pose,KeyFrameEstimzationDOF dof=UPDATE_KF_SIM3,double* information=NULL);

    // Fit the sim3 transform between 2 synchronized trajectory
    virtual bool fitSim3(const std::vector<std::pair<GSLAM::SE3, GSLAM::SE3> > &matches,// T_{12}
                         GSLAM::SIM3& sim3,KeyFrameEstimzationDOF dof=UPDATE_KF_SIM3,double* information=NULL){return false;}

    // MAPPING: Do bundle adjust with auto calibration or not: BUNDLEADJUST, INVDEPTH_BUNDLE, POSEGRAPH
    virtual bool optimize(BundleGraph& graph);
    virtual bool magin(BundleGraph& graph){return false;}// Convert bundle graph to pose graph

};


ceres::Solver::Options getOption(const OptimzeConfig& _config)
{
    ceres::Solver::Options ceres_config_options;
    ceres_config_options.max_num_iterations = _config.maxIterations;
    ceres_config_options.preconditioner_type = ceres::JACOBI;
    ceres_config_options.minimizer_progress_to_stdout = _config.verbose;
    ceres_config_options.logging_type = ceres::SILENT;
    ceres_config_options.parameter_tolerance = 1e-8;
    ceres_config_options.num_threads=4;
    //ceres_config_options.num_linear_solver_threads=4;
    ceres_config_options.max_solver_time_in_seconds=svar.GetDouble("Optimizer.MaxSolverTime",1e3);
    // If Sparse linear solver are available
    // Descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
    if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
    {
        ceres_config_options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        ceres_config_options.linear_solver_type  = ceres::SPARSE_SCHUR;
    }
    else if(ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
    {
        ceres_config_options.sparse_linear_algebra_library_type = ceres::CX_SPARSE;
        ceres_config_options.linear_solver_type  = ceres::SPARSE_SCHUR;
    }
#if CERES_VERSION_MINOR>=12
    else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
    {
        ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
        ceres_config_options.linear_solver_type  = ceres::SPARSE_SCHUR;
    }
#endif
    return ceres_config_options;
}


// FIXME: output of information
bool OptimizerAutoDiffCeres::optimizePose(std::vector<std::pair<CameraAnchor,CameraAnchor> >&       matches,
                                  std::vector<IdepthEstimation>& firstIDepth,
                                  GSLAM::SE3&    relativePose,
                                  KeyFrameEstimzationDOF dof,double* information)
{
    if(matches.size()<5||matches.size()!=firstIDepth.size()) return false;

    ceres::Problem problem;
#ifdef USE_PARAMENTERIZATION
    auto invPose=relativePose.inverse();
    double* se3=(double*)&invPose;
#else
    auto se3=relativePose.inverse().ln();
#endif

    // Set a LossFunction to be less penalized by false measurements
    //  - set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction * p_LossFunction =
            true ?
                new ceres::HuberLoss(_config.projectErrorHuberThreshold*_config.projectErrorHuberThreshold)
              : nullptr;

    for(int i=0;i<matches.size();i++)
    {
        double* match=(double*)&matches[i];
        double* idepth=(double*)&firstIDepth[i];

#ifdef USE_PARAMENTERIZATION
        ceres::CostFunction* costFunction=new InvDepthRelativePoseParameterizationCostFunction(match);
        problem.AddResidualBlock(costFunction,p_LossFunction,idepth,se3);
#else
        ceres::CostFunction* costFunction=new InvDepthRelativePoseCostFunction(match);
        problem.AddResidualBlock(costFunction,
                                 p_LossFunction,
                                 idepth,(double*)&se3);
#endif
        if(firstIDepth[i].y==0)
            problem.SetParameterBlockConstant(idepth);
    }

#ifdef USE_PARAMENTERIZATION
    problem.SetParameterization(se3,new ceres::ProductParameterization(new ceres::QuaternionParameterization(),
                                                                                     new ceres::IdentityParameterization(3)));
#endif


    auto ceres_config_options=getOption(_config);
    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_config_options, &problem, &summary);
    if (ceres_config_options.minimizer_progress_to_stdout)
        std::cout << summary.FullReport() << std::endl;

    // If no error, get back refined parameters
#if CERES_VERSION_MINOR>=12
    if (!summary.IsSolutionUsable())
#else
    //if(summary.error.size())
    if( !summary.IsSolutionUsable() )
#endif
    {
        if (ceres_config_options.minimizer_progress_to_stdout)
            std::cout << "OptimizePose failed." << std::endl;
        return false;
    }
    else // Solution is usable
    {
        if (ceres_config_options.minimizer_progress_to_stdout)
        {
            // Display statistics about the minimization
            std::cout << std::endl
                      << " #residuals: " << summary.num_residuals << "\n"
                      << " Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals) << "\n"
                      << " Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals) << "\n"
                      << " Time (s): " << summary.total_time_in_seconds << "\n"
                      << std::endl;
        }
#ifdef USE_PARAMENTERIZATION
        relativePose=invPose.inverse();
#else
        relativePose=GSLAM::SE3::exp(se3).inverse();
#endif
    }

    return true;
}

// Update pose with 3D-2D corrospondences
bool OptimizerAutoDiffCeres::optimizePnP(const std::vector<std::pair<GSLAM::Point3d,CameraAnchor> >& matches,
                                 GSLAM::SE3& pose,
                                 KeyFrameEstimzationDOF dof,
                                 double* information)
{
    if(matches.size()<3) return false;
    ceres::Problem problem;
    auto se3=pose.inverse().ln();

    // Set a LossFunction to be less penalized by false measurements
    //  - set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction * p_LossFunction =
            true ?
                new ceres::HuberLoss(_config.projectErrorHuberThreshold*_config.projectErrorHuberThreshold)
              : nullptr;

    for(int i=0;i<matches.size();i++)
    {
        double* match=(double*)&matches[i];

        ceres::CostFunction* costFunction=new IdeaCameraPnPCostFunction(match);//IdeaProjectResiduals::create(match+3,match);
        problem.AddResidualBlock(costFunction,
                                 p_LossFunction,
                                 (double*)&se3);
    }

    // Solve
    ceres::Solver::Summary summary;
    auto ceres_config_options=getOption(_config);
    ceres::Solve(ceres_config_options, &problem, &summary);
    if (ceres_config_options.minimizer_progress_to_stdout)
        std::cout << summary.FullReport() << std::endl;

    // If no error, get back refined parameters
#if CERES_VERSION_MINOR>=12
    if (!summary.IsSolutionUsable())
#else
    //if(summary.error.size())
    if( !summary.IsSolutionUsable() )
#endif
    {
        if (ceres_config_options.minimizer_progress_to_stdout)
            std::cout << "OptimizePose failed." << std::endl;
        return false;
    }
    else // Solution is usable
    {
        if (ceres_config_options.minimizer_progress_to_stdout)
        {
            // Display statistics about the minimization
            std::cout << std::endl
                      << " #residuals: " << summary.num_residuals << "\n"
                      << " Initial RMSE: " << std::sqrt( summary.initial_cost / summary.num_residuals) << "\n"
                      << " Final RMSE: " << std::sqrt( summary.final_cost / summary.num_residuals) << "\n"
                      << " Time (s): " << summary.total_time_in_seconds << "\n"
                      << std::endl;
        }
        pose=GSLAM::SE3::exp(se3).inverse();
    }

    return true;
}

bool OptimizerAutoDiffCeres::optimizeICP(const std::vector<std::pair<Point3d, Point3d> > &matches,
                                 SIM3 &pose, KeyFrameEstimzationDOF dof, double *information)
{
    if(matches.size()<3) return false;
    auto se3=pose.get_se3().ln();
    Eigen::Matrix<double,7,1> sim3;
    for(int i=0;i<6;i++) sim3(i)=se3.data[i];
    sim3(6)=pose.get_scale();
    ceres::Problem problem;

    // Set a LossFunction to be less penalized by false measurements
    //  - set it to NULL if you don't want use a lossFunction.
    ceres::LossFunction * p_LossFunction =
            true ?
                new ceres::HuberLoss(svar.GetDouble("ICPHuberFactor",100.))
              : nullptr;

    for(int i=0;i<matches.size();i++)
    {
        double* match=(double*)&matches[i];

        ceres::CostFunction* costFunction=new EdgePointICPSim3CostFunction(match);
        problem.AddResidualBlock(costFunction,
                                 p_LossFunction,
                                 (double*)&sim3);
    }

    // Solve
    ceres::Solver::Summary summary;
    auto ceres_config_options=getOption(_config);
    ceres::Solve(ceres_config_options, &problem, &summary);
    if (ceres_config_options.minimizer_progress_to_stdout)
        std::cout << summary.FullReport() << std::endl;

    for(int i=0;i<6;i++) se3.data[i]=sim3(i);
    pose.get_se3()=GSLAM::SE3::exp(se3);
    pose.get_scale()=sim3(6);
    return true;
}

bool OptimizerAutoDiffCeres::optimize(BundleGraph &graph)
{
    // check edge types
    int graphType=(graph.mappointObserves.size()?1:0)
            +(graph.invDepthObserves.size()?2:0)
            +(graph.se3Graph.size()?4:0)
            +(graph.sim3Graph.size()?8:0)
            +(graph.gpsGraph.size()?16:0);
    if(graphType==0)
    {
        LOG(ERROR)<<"Graph is empty!";
        return false;
    }

    ceres::Problem problem;
    // add verticals
    std::vector<double> kfParas(7*graph.keyframes.size());
    if(graph.mappointObserves.size())
    {

    }
    if(graph.invDepthObserves.size())
    {

    }

    for(int i=0;i<graph.keyframes.size();i++)
    {
        KeyFrameEstimzation& keyframe=graph.keyframes[i];
        double *para=&kfParas[i*7];
        *(pi::Array_<double,6>*)para=keyframe.estimation.get_se3().ln();
        para[6]=keyframe.estimation.get_scale();
        switch (keyframe.dof) {
        case UPDATE_KF_NONE:// fixed
            problem.AddParameterBlock(para,7);
            problem.SetParameterBlockConstant(para);
            break;
        case UPDATE_KF_SIM3:// SIM3 vertical
            problem.AddParameterBlock(para,7);
            break;
//        case UPDATE_KF_SE3:
////            if(para[6]==1.)
////                problem.AddParameterBlock(para,6);// SE3 vertical
////            else
//                problem.AddParameterBlock(para,7);
//            break;
        default:
        {
            problem.AddParameterBlock(para,7);
            std::vector<int> vec_constant;
            for(int i=0;i<7;i++)
                if(!(keyframe.dof&(1<<i))) vec_constant.push_back(i);
            ceres::SubsetParameterization *subset_parameterization =
                    new ceres::SubsetParameterization(7, vec_constant);
            problem.SetParameterization(para, subset_parameterization);
        }
            break;
        }
    }

    ceres::LossFunction * projectLoss =
            true ?
                new ceres::HuberLoss(_config.projectErrorHuberThreshold*_config.projectErrorHuberThreshold)
              : nullptr;

    if((!graph.camera.isValid())||graph.camera.CameraType()=="Ideal")
    {
        for(BundleEdge& edge:graph.mappointObserves)
        {
            problem.AddResidualBlock(new IdeaProjectMapPointSIM3BundleCostFunction((double*)&edge.measurement),
                                     projectLoss,(double*)&graph.mappoints[edge.pointId].first,&kfParas[edge.frameId*7]);
        }

        for(BundleEdge& edge:graph.invDepthObserves)
        {
            InvDepthEstimation& idepth=graph.invDepths[edge.pointId];
            problem.AddResidualBlock(new IdeaProjectInvDepthSIM3BundleCostFunction((double*)&idepth.anchor,(double*)&edge.measurement),
                                     projectLoss,&kfParas[idepth.frameId*7],&kfParas[edge.frameId*7],(double*)&idepth.estimation);
        }
    }
//    else
//    {
//        for(BundleEdge& edge:graph.mappointObserves)
//        {
//            problem.AddResidualBlock(new IdeaProjectMapPointSIM3BundleCostFunction((double*)&edge.measurement),
//                                     projectLoss,(double*)&graph.mappoints[edge.pointId].first,&kfParas[edge.frameId*7]);
//        }
//    }

    for(SE3Edge& edge:graph.se3Graph)
    {
//        if(graph.keyframes[edge.firstId].dof==UPDATE_KF_SIM3&&graph.keyframes[edge.secondId].dof==UPDATE_KF_SIM3)
            problem.AddResidualBlock(new EdgeSE3CostFunction((double*)&edge.measurement,edge.information),
                                     NULL,&kfParas[edge.firstId*7],&kfParas[edge.secondId*7]);
//        else if(graph.keyframes[edge.firstId].dof==UPDATE_KF_SE3&&graph.keyframes[edge.secondId].dof==UPDATE_KF_SE3)
//            problem.AddResidualBlock(new EdgeSE3CostFunction((double*)&edge.measurement,edge.information),
//                                     NULL,&kfParas[edge.firstId*7],&kfParas[edge.secondId*7]);
//        else LOG(ERROR)<<"A SE3Edge should link two SE3 or SIM3 verticals.";
    }

    for(SIM3Edge& edge:graph.sim3Graph)
    {
        problem.AddResidualBlock(new EdgeSIM3CostFunction((double*)&edge.measurement,edge.information),
                                 NULL,&kfParas[edge.firstId*7],&kfParas[edge.secondId*7]);
    }

    for(GPSEdge& edge:graph.gpsGraph)
    {
        problem.AddResidualBlock(new EdgeGPSCostFunction((double*)&edge.measurement,edge.information),
                                 NULL,&kfParas[edge.frameId*7]);
    }

    // Solve
    ceres::Solver::Summary summary;
    auto ceres_config_options=getOption(_config);
    ceres::Solve(ceres_config_options, &problem, &summary);
    if (ceres_config_options.minimizer_progress_to_stdout)
        std::cout << summary.FullReport() << std::endl;

    for(int i=0;i<graph.keyframes.size();i++)
    {
        KeyFrameEstimzation& keyframe=graph.keyframes[i];
        double *para=&kfParas[i*7];
        keyframe.estimation.get_se3()=SE3::exp(*(pi::Array_<double,6>*)para);
        keyframe.estimation.get_scale()=para[6];
    }
    return true;
}

USE_OPTIMIZER_PLUGIN(OptimizerAutoDiffCeres);
