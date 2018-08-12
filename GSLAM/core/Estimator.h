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
// Author: heyu.nwpu@gmail.com (He Yu)
//
// This is the GSLAM main API header

#ifndef GSLAM_CORE_ESTIMATOR_H_
#define GSLAM_CORE_ESTIMATOR_H_

#include <string>
#include <vector>
#include "GSLAM.h"

#define USE_ESTIMATOR_PLUGIN(EST_CLASS)              \
  extern "C" {                                       \
  SPtr<GSLAM::Estimator> createEstimatorInstance() { \
    return SPtr<GSLAM::Estimator>(new EST_CLASS());  \
  }                                                  \
  }

namespace GSLAM {

class Estimator;
typedef SPtr<Estimator> (*funcCreateEstimatorInstance)();

enum EstimatorMethod {
  ITERATIVE = 0,
  FM_7POINT = 1,  //!< 7-point algorithm
  FM_8POINT = 2,  //!< 8-point algorithm
  LMEDS = 4,      //!< least-median algorithm
  RANSAC = 8      //!< RANSAC algorithm
};

class Estimator : public GObject {
 public:
  Estimator() {}

  virtual ~Estimator() {}

  virtual std::string type() const { return "Estimator"; }

  // 2D corrospondences
  virtual bool findHomography(double* H,  // 3x3 dof=8
                              const std::vector<Point2d>& srcPoints,
                              const std::vector<Point2d>& dstPoints,
                              int method = 0, double ransacReprojThreshold = 3,
                              std::vector<uchar>* mask = NULL) const {
    return false;
  }

  virtual bool findAffine2D(double* A,  // 2X3
                            const std::vector<Point2d>& srcPoints,
                            const std::vector<Point2d>& dstPoints,
                            bool fullAffine = true) const {
    return false;
  }

  virtual bool findFundamental(double* F,  // 3x3
                               const std::vector<Point2d>& points1,
                               const std::vector<Point2d>& points2,
                               int method = 0, double param1 = 3.,
                               double param2 = 0.99,
                               std::vector<uchar>* mask = NULL) const {
    return false;
  }

  virtual bool findEssentialMatrix(double* E,  // 3x3 dof=5
                                   const std::vector<Point2d>& points1,
                                   const std::vector<Point2d>& points2,
                                   int method = 0, double param1 = 0.01,
                                   double param2 = 0.99,
                                   std::vector<uchar>* mask = NULL) const {
    return false;
  }

  // 3D corrospondences
  virtual bool findSIM3(const SIM3& S, const std::vector<Point3d>& from,
                        const std::vector<Point3d>& to, int method = 0,
                        double ransacThreshold = -1,
                        std::vector<uchar>* mask = NULL) const {
    return false;
  }

  virtual bool findAffine3D(double* A, const std::vector<Point3d>& src,
                            const std::vector<Point3d>& dst,
                            std::vector<int>* inliers = NULL,
                            double ransacThreshold = 3,
                            double confidence = 0.99) const {
    return false;
  }

  virtual bool findPlane(const SE3& plane, const std::vector<Point3d>& points,
                         int method = 0, double ransacThreshold = -1.,
                         std::vector<uchar>* mask = NULL) const {
    return false;
  }

  // 2D&3D corrospondences
  virtual bool findPnPRansac(
      const SE3& world2camera, const std::vector<Point3d>& objectPoints,
      const std::vector<Point2d>& imagePoints, const GSLAM::Camera& camera,
      bool useExtrinsicGuess = false, int iterationsCount = 100,
      float reprojectionError = 8.0, int minInliersCount = 100,
      std::vector<int>* inliers = NULL, int flags = ITERATIVE) const {
    return false;
  }

  virtual bool trianglate(
      const SE3& ref2cur,
      const Point3d& refDirection,  // camera.UnProject(ref2d)
      const Point3d& curDirection,  // camera.UnProject(cur2d)
      const Point3d& refPt) const {
    return false;
  }

  static SPtr<Estimator> create(std::string pluginName = "") {
    if (pluginName.empty()) {
      pluginName = svar.GetString("EstimatorPlugin", "libgslam_estimator");
    }
    SPtr<SharedLibrary> plugin = Registry::get(pluginName);
    if (!plugin) return SPtr<Estimator>();
    funcCreateEstimatorInstance createFunc =
        (funcCreateEstimatorInstance)plugin->getSymbol(
            "createEstimatorInstance");
    if (!createFunc)
      return SPtr<Estimator>();
    else
      return createFunc();
  }
};
}  // namespace GSLAM
#endif  // GSLAM_CORE_ESTIMATOR_H_
