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

#include <string>

#ifndef GSLAM_CORE_EVENT_H_
#define GSLAM_CORE_EVENT_H_
#include "GSLAM.h"

namespace GSLAM {
#define REG_EVENT(EVENTNAME, DATAFORMAT, DATANAME)                            \
  class EVENTNAME : public GEvent {                                           \
   public:                                                                    \
    virtual std::string type() const { return #EVENTNAME; }                   \
    explicit EVENTNAME(const DATAFORMAT& DATANAME) : _##DATANAME(DATANAME) {} \
    DATAFORMAT _##DATANAME;                                                   \
  }

#define REG_EVENT2(E, D1, N1, D2, N2)                       \
  class E : public GEvent {                                 \
   public:                                                  \
    virtual std::string type() const { return #E; }         \
    E(const D1& N1, const D2& N2) : _##N1(N1), _##N2(N2) {} \
    D1 _##N1;                                               \
    D2 _##N2;                                               \
  }

class GEvent : public GObject {
 public:
  GEvent() : _handled(false), _time(0.) {}

  virtual std::string type() const { return "GEvent"; }

  /** Set whether this event has been handled by an event handler or not.*/
  void setHandled(bool handled = true) const { _handled = handled; }

  /** Get whether this event has been handled by an event handler or not.*/
  bool getHandled() const { return _handled; }

  /** set time in seconds of event. */
  void setTime(double time) { _time = time; }

  /** get time in seconds of event. */
  double getTime() const { return _time; }

 protected:
  virtual ~GEvent() {}
  mutable bool _handled;
  double _time;
};

REG_EVENT(CommandEvent, std::string, cmd);
REG_EVENT(ScenceCenterEvent, Point3d, center);
REG_EVENT(ScenceRadiusEvent, double, radius);
REG_EVENT(SetViewPoseEvent, SE3, pose);
REG_EVENT(CurrentFrameEvent, FramePtr, frame);
REG_EVENT2(DrawableEvent, GObjectPtr, obj, std::string, name);
REG_EVENT2(DebugImageEvent, GImage, img, std::string, name);
}  // namespace GSLAM
#endif  // GSLAM_CORE_EVENT_H_
