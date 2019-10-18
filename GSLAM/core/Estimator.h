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
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: zd5945@126.com (Yong Zhao)
//
// Estimator of GSLAM aims to provide a collection of close-form solvers covering all interesting cases with robust sample consensus (RANSAC) methods.

#ifndef GSLAM_CORE_ESTIMATOR_H_
#define GSLAM_CORE_ESTIMATOR_H_

#include <string>
#include <vector>
#include "GSLAM/core/SIM3.h"
#include "GSLAM/core/Svar.h"
#include "GSLAM/core/Registry.h"

#define USE_ESTIMATOR_PLUGIN(EST_CLASS)                                  \
  extern "C" GSLAM::Estimator* createEstimatorInstance() {          \
    return new EST_CLASS();                      \
  }                                                                      \
  class EST_CLASS##_Register {                                           \
   public:                                                               \
    EST_CLASS##_Register() {                                             \
      GSLAM::Estimator::buildinEstimators()\
.Set<funcCreateEstimatorInstance>\
("Default",createEstimatorInstance);                                       \
    }                                                                    \
  } EST_CLASS##_Register_instance;

namespace GSLAM {

class Estimator;
typedef std::shared_ptr<Estimator> EstimatorPtr;
typedef Estimator* (*funcCreateEstimatorInstance)();
typedef Matrix3d Homography2D;
typedef Matrix<double,2,3> Affine2D;
typedef Matrix3d Fundamental;
typedef Fundamental Essential;
typedef Matrix<double,3,4> Affine3D;
typedef unsigned char uchar;

enum EstimatorMethod {
    MODEL_METHOD=0xFF,
    F8_Point,
    F7_Point,
    E5_Stewenius,
    E5_Nister,
    E5_Kneip,
    H4_Point,
    A3_Point,
    P4_EPnP,
    P3_Gao,
    P3_Kneip,
    P3_GPnP,
    P3_ITERATIVE,
    P2_Kneip,
    T2_Triangulate,
    A4_Point,
    S3_Horn,
    P3_Plane,
    SAMPLE_METHOD=0xF00,
    RANSAC  =  0<<8,      //!< RANSAC algorithm
    LMEDS   =  1<<8,      //!< least-median algorithm
    NOSAMPLE=  2<<8
};

class Estimator  {
 public:
  Estimator() {}

  virtual ~Estimator() {}

  virtual std::string type() const { return "Estimator"; }

  // 2D corrospondences
  virtual bool findHomography(Homography2D* H,  // 3x3 dof=8
                              const std::vector<Point2d>& srcPoints,
                              const std::vector<Point2d>& dstPoints,
                              int    method = H4_Point&RANSAC,
                              double threshold = 3,
                              double confidence = 0.99,
                              std::vector<uchar>* mask = NULL) const = 0;

  virtual bool findAffine2D(Affine2D* A,  // 2X3
                            const std::vector<Point2d>& srcPoints,
                            const std::vector<Point2d>& dstPoints,
                            int method = A3_Point&RANSAC,
                            double threshold = 3,
                            double confidence = 0.99,
                            std::vector<uchar>* mask = NULL) const =0;

  virtual bool findFundamental(Fundamental* F,  // 3x3
                               const std::vector<Point2d>& points1,
                               const std::vector<Point2d>& points2,
                               int method = F8_Point&RANSAC,
                               double threshold = 3.,
                               double confidence = 0.99,
                               std::vector<uchar>* mask = NULL) const = 0;

  virtual bool findEssentialMatrix(Essential* E,  // 3x3 dof=5
                                   const std::vector<Point2d>& points1,
                                   const std::vector<Point2d>& points2,
                                   int method = E5_Nister&RANSAC,
                                   double threshold = 0.01,
                                   double confidence = 0.99,
                                   std::vector<uchar>* mask = NULL) const  = 0;

  // 3D corrospondences
  virtual bool findSIM3(SIM3* S,
                        const std::vector<Point3d>& from,
                        const std::vector<Point3d>& to,
                        int    method = S3_Horn&RANSAC,
                        double threshold = 0.01,
                        double confidence = 0.99,
                        std::vector<uchar>* mask = NULL) const = 0;

  virtual bool findAffine3D(Affine3D* A,
                            const std::vector<Point3d>& src,
                            const std::vector<Point3d>& dst,
                            int    method = A4_Point&RANSAC,
                            double threshold =  0.01,
                            double confidence = 0.99,
                            std::vector<uchar>* mask = NULL) const = 0;

  virtual bool findPlane(SE3* plane,
                         const std::vector<Point3d>& points,  // NOLINT
                         int    method = P3_Plane&RANSAC,
                         double threshold =  0.01,
                         double confidence = 0.99,
                         std::vector<uchar>* mask = NULL) const = 0;

  // 2D&3D corrospondences
  virtual bool findPnP( SE3* world2camera,
                        const std::vector<Point3d>& objectPoints,
                        const std::vector<Point2d>& imagePoints,
                        int    method = P3_ITERATIVE&RANSAC,
                        double threshold =  0.01,
                        double confidence = 0.99,
                        std::vector<uchar>* mask = NULL) const = 0;

  virtual bool trianglate(Point3d* refPt,
      const SE3& ref2cur,
      const Point3d& refDirection,  // camera.UnProject(ref2d)
      const Point3d& curDirection) const  = 0;  // camera.UnProject(cur2d)

  static Svar& buildinEstimators(){
      static Svar var=Svar::object();return var;
  }

  static EstimatorPtr create(std::string pluginName = "") {
    funcCreateEstimatorInstance createFunc =buildinEstimators()
        .Get<funcCreateEstimatorInstance>("Default",NULL);
    if (createFunc) return EstimatorPtr(createFunc());

    if (pluginName.empty()) {
      pluginName = svar.GetString("EstimatorPlugin", "libgslam_estimator");
    }
    SharedLibraryPtr plugin = Registry::get(pluginName);
    if (!plugin) return EstimatorPtr();
    createFunc = (funcCreateEstimatorInstance)plugin->getSymbol(
        "createEstimatorInstance");
    if (!createFunc)
      return EstimatorPtr();
    else
      return EstimatorPtr(createFunc());
  }
};
}  // namespace GSLAM
#endif  // GSLAM_CORE_ESTIMATOR_H_
