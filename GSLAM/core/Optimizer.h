// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAYongMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: 18392360048@163.com (huboni)
//
// This is the GSLAM main API header

#ifndef GSLAM_CORE_OPTIMIZER_H_
#define GSLAM_CORE_OPTIMIZER_H_

#include <string>
#include <utility>
#include <vector>
#include "GSLAM.h"
#define USE_OPTIMIZER_PLUGIN(OPT_CLASS)              \
  extern "C" {                                       \
  SPtr<GSLAM::Optimizer> createOptimizerInstance() { \
    return SPtr<GSLAM::Optimizer>(new OPT_CLASS());  \
  }                                                  \
  }

namespace GSLAM {

class Optimizer;
typedef SPtr<Optimizer> (*funcCreateOptimizerInstance)();

enum CameraProjectionType {
  PROJECTION_PINHOLE,  // z = 1
  PROJECTION_SPHERE    // ||x,y,z||=1
};

enum InvDepthEstimationDOF {
  UPDATE_ID_NONE = 0,
  UPDATE_ID_IDEPTH = 1,
  UPDATE_ID_SIGMA = 2,
  UPDATE_ID_IDEPTHSIGMA = UPDATE_ID_IDEPTH | UPDATE_ID_SIGMA
};

enum KeyFrameEstimzationDOF {
  UPDATE_KF_NONE = 0,
  UPDATE_KF_X = 1,
  UPDATE_KF_Y = 2,
  UPDATE_KF_Z = 4,
  UPDATE_KF_RX = 8,
  UPDATE_KF_RY = 16,
  UPDATE_KF_RZ = 32,
  UPDATE_KF_SCALE = 64,
  UPDATE_KF_TRANSLATION = UPDATE_KF_X | UPDATE_KF_Y | UPDATE_KF_Z,
  UPDATE_KF_XYZ = UPDATE_KF_TRANSLATION,
  UPDATE_KF_ROTATION = UPDATE_KF_RX | UPDATE_KF_RY | UPDATE_KF_RZ,
  UPDATE_KF_SE3 = UPDATE_KF_TRANSLATION | UPDATE_KF_ROTATION,
  UPDATE_KF_SIM3 = UPDATE_KF_SE3 | UPDATE_KF_SCALE
};

enum CameraEstimationDOF {
  UPDATE_CAMERA_NONE = 0,
  UPDATE_CAMERA_FOCAL = 1,
  UPDATE_CAMERA_CENTER = 2,
  UPDATE_CAMERA_K1 = 4,
  UPDATE_CAMERA_K2 = 8,
  UPDATE_CAMERA_P1 = 16,
  UPDATE_CAMERA_P2 = 32,
  UPDATE_CAMERA_K3 = 64,
  UPDATE_CAMERA_K = UPDATE_CAMERA_K1 | UPDATE_CAMERA_K2 | UPDATE_CAMERA_K3,
  UPDATE_CAMERA_P = UPDATE_CAMERA_P1 | UPDATE_CAMERA_P2,
  UPDATE_CAMERA_KP = UPDATE_CAMERA_K | UPDATE_CAMERA_P,
  UPDATE_CAMERA_PINHOLE = UPDATE_CAMERA_FOCAL | UPDATE_CAMERA_CENTER,
  UPDATE_CAMERA_FISHEYE = UPDATE_CAMERA_PINHOLE | UPDATE_CAMERA_KP
};

typedef GSLAM::Point3d
    CameraAnchor;  // for both Pinhole projection and Sphere projection
typedef GSLAM::Point2d IdepthEstimation;  // [idepth,sigma]^T

struct InvDepthEstimation {
  GSLAM::PointID frameId;
  CameraAnchor anchor;
  IdepthEstimation estimation;  // idepth,sigma
  InvDepthEstimationDOF dof;
};

typedef std::pair<GSLAM::Point3d, bool>
    MapPointEstimation;  // false:FIX true:NOT FIXED

struct KeyFrameEstimzation {
  GSLAM::SIM3 estimation;  // T_{wc} : the transform from camera to world
  KeyFrameEstimzationDOF dof;
};

struct BundleEdge {
  GSLAM::FrameID pointId, frameId;
  CameraAnchor measurement;
  double* information;  // 2*2
};

// Consider the first frame as origin, transform of the second frame should be
// SE3_{12}
struct SE3Edge {
  GSLAM::FrameID firstId, secondId;
  GSLAM::SE3 measurement;  // SE3_{12}:=SE3_1^{-1}*SE3_2 =>
  // P_{world}=SE3_2*P_{C2}=SE3_1*SE3_{12}*P_{C2}=SE3_1*P_{C1}
  double* information;  // 6*6
};

// Consider the first frame as origin, transform of the second frame should be
// SIM3_{12}
struct SIM3Edge {
  GSLAM::FrameID firstId, secondId;
  GSLAM::SIM3 measurement;  // SIM3_{12}:=SIM3_1^{-1}*SIM3_2
  double* information;      // 7*7
};

struct GPSEdge {
  GSLAM::FrameID frameId;
  GSLAM::SE3 measurement;  // SE3_{gps}:=SE3_{frameId}
  double* information;     // 6*6
};

struct BundleGraph {
  // VERTICALS : Estimations
  std::vector<InvDepthEstimation>
      invDepths;  // mappoint representation with inverse depth
  std::vector<MapPointEstimation>
      mappoints;  // mappoint representation with [x,y,z]
  std::vector<KeyFrameEstimzation>
      keyframes;  // keyframe representation with SIM3:[rx,ry,rz,w,x,y,z,scale]

  // EDGES : Residual blocks from measurements
  std::vector<BundleEdge> invDepthObserves,
      mappointObserves;  // [pt_id,kf_id],[x,y,z]
  std::vector<SE3Edge>
      se3Graph;  // [firstId,secondId,SE3_{12}]  SE3_{12}:=SE3_1^{-1}*SE3_2
  std::vector<SIM3Edge>
      sim3Graph;  // [firstId,secondId,SIM3_{12}] SIM3_{12}:=SIM3_1^{-1}*SIM3_2
  std::vector<GPSEdge>
      gpsGraph;  // [frameId,SE3_{gps}]          SE3_{gps}:=SE3_{frameId}

  // CAMERA : Invalid camera indicates idea camera
  GSLAM::Camera camera;
  CameraEstimationDOF cameraDOF;
};

struct OptimzeConfig {
  CameraProjectionType cameraProjectionType = PROJECTION_PINHOLE;

  double projectErrorHuberThreshold = 0.01;

  int maxIterations = 500;

  bool verbose = false;
};

class Optimizer {
 public:
  explicit Optimizer(OptimzeConfig config = OptimzeConfig())
      : _config(config) {}

  virtual ~Optimizer() {}

  // TRACKING: Update relative pose agaist the first frame with known or unknown
  // depth
  virtual bool optimizePose(
      const std::vector<std::pair<CameraAnchor, CameraAnchor> >& matches,
      const std::vector<IdepthEstimation>& firstIDepth,
      const GSLAM::SE3& relativePose,  // T_{12}
      KeyFrameEstimzationDOF dof = UPDATE_KF_SE3, double* information = NULL) {
    return false;
  }

  // Update pose with 3D-2D corrospondences
  virtual bool optimizePnP(
      const std::vector<std::pair<GSLAM::Point3d, CameraAnchor> >& matches,
      const GSLAM::SE3& pose, KeyFrameEstimzationDOF dof = UPDATE_KF_SE3,
      double* information = NULL) {
    return false;
  }

  // Update pose with 3D-3D corrospondences
  virtual bool optimizeICP(
      const std::vector<std::pair<GSLAM::Point3d, GSLAM::Point3d> >&
          matches,  // T_{12}
      const GSLAM::SIM3& pose,
      KeyFrameEstimzationDOF dof = UPDATE_KF_SE3, double* information = NULL) {
    return false;
  }

  // Fit the sim3 transform between 2 synchronized trajectory
  virtual bool fitSim3(
      const std::vector<std::pair<GSLAM::SE3, GSLAM::SE3> >& matches,  // T_{12}
      const GSLAM::SIM3& sim3, KeyFrameEstimzationDOF dof = UPDATE_KF_SIM3,
      double* information = NULL) {
    return false;
  }

  // MAPPING: Do bundle adjust with auto calibration or not: BUNDLEADJUST,
  // INVDEPTH_BUNDLE, POSEGRAPH
  virtual bool optimize(const BundleGraph& graph) { return false; }
  virtual bool magin(const BundleGraph& graph) {
    return false;
  }  // Convert bundle graph to pose graph

  static SPtr<Optimizer> create(std::string pluginName = "") {
    if (pluginName.empty()) {
      pluginName = svar.GetString("OptimizerPlugin", "libgslam_optimizer");
    }
    SPtr<SharedLibrary> plugin = Registry::get(pluginName);
    if (!plugin) return SPtr<Optimizer>();
    funcCreateOptimizerInstance createFunc =
        (funcCreateOptimizerInstance)plugin->getSymbol(
            "createOptimizerInstance");
    if (!createFunc)
      return SPtr<Optimizer>();
    else
      return createFunc();
  }

  OptimzeConfig _config;
};
}  // namespace GSLAM
#endif  // GSLAM_CORE_OPTIMIZER_H_
