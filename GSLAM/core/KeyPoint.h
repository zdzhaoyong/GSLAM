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

#ifndef GSLAM_CORE_KEYPOINT_H_
#define GSLAM_CORE_KEYPOINT_H_

#include "Point.h"

namespace GSLAM {

class KeyPoint {
 public:
  KeyPoint()
      : pt(0, 0), size(0), angle(-1), response(0), octave(0), class_id(-1) {}

  KeyPoint(Point2f _pt, float _size, float _angle = -1, float _response = 0,
           int _octave = 0, int _class_id = -1)
      : pt(_pt),
        size(_size),
        angle(_angle),
        response(_response),
        octave(_octave),
        class_id(_class_id) {}

  KeyPoint(float x, float y, float _size, float _angle = -1,
           float _response = 0, int _octave = 0, int _class_id = -1)
      : pt(x, y),
        size(_size),
        angle(_angle),
        response(_response),
        octave(_octave),
        class_id(_class_id) {}

#ifdef __OPENCV_FEATURES_2D_HPP__
  explicit KeyPoint(const cv::KeyPoint& kp)
      : pt(kp.pt.x, kp.pt.y),
        size(kp.size),
        angle(kp.angle),
        response(kp.response),
        octave(kp.octave),
        class_id(kp.class_id) {}

  operator cv::KeyPoint() const {
    return cv::KeyPoint(pt.x, pt.y, size, angle, response, octave, class_id);
  }
#endif
  typedef union Cv32suf {
    int i;
    unsigned u;
    float f;
  } Cv32suf;

  size_t hash() const {
    size_t _Val = 2166136261U, scale = 16777619U;
    Cv32suf u;
    u.f = pt.x;
    _Val = (scale * _Val) ^ u.u;
    u.f = pt.y;
    _Val = (scale * _Val) ^ u.u;
    u.f = size;
    _Val = (scale * _Val) ^ u.u;
    u.f = angle;
    _Val = (scale * _Val) ^ u.u;
    u.f = response;
    _Val = (scale * _Val) ^ u.u;
    _Val = (scale * _Val) ^ ((size_t)octave);
    _Val = (scale * _Val) ^ ((size_t)class_id);
    return _Val;
  }

  Point2f pt;  //!< coordinates of the keypoints
  float size;  //!< diameter of the meaningful keypoint neighborhood
  float
      angle;  //!< computed orientation of the keypoint (-1 if not applicable);
              //!< it's in [0,360) degrees and measured relative to
              //!< image coordinate system, ie in clockwise.
  float response;  //!< the response by which the most strong keypoints have
                   //! been selected. Can be used for the further sorting or
  //! subsampling
  int octave;    //!< octave (pyramid layer) from which the keypoint has been
                 //! extracted
  int class_id;  //!< object class (if the keypoints need to be clustered by an
                 //! object they belong to)
};
}  // namespace GSLAM

#endif  // GSLAM_CORE_KEYPOINT_H_
