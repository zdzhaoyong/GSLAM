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

#ifndef GSLAM_CORE_MAPFUSION_H_
#define GSLAM_CORE_MAPFUSION_H_
#include <GSLAM/core/GSLAM.h>
#include <string>

#define USE_MAPFUSION_PLUGIN(C)                   \
  extern "C" {                                    \
  GSLAM::MapFusionPtr createMapFusionInstance() { \
    return GSLAM::MapFusionPtr(new C());          \
  }                                               \
  }

namespace GSLAM {

class MapFusion;
typedef SPtr<MapFusion> MapFusionPtr;
typedef GSLAM::MapFusionPtr (*funcCreateMapFusionInstance)();

class MapFusion : public GObject {
 public:
  MapFusion() : cbk(NULL) {}

  virtual std::string type() const { return "MapFusionNone"; }

  virtual bool valid() { return false; }  // whether if this Map2DFusion usable
  virtual void draw() {}                  // Plot things with OpenGL
  virtual bool feed(FramePtr frame) { return false; }
  virtual bool feed(MapPtr map) { return false; }
  virtual bool save(const std::string& filePath) { return false; }

  virtual bool setCallBack(GObjectHandle* callback = NULL) {
    cbk = callback;
    return true;
  }

  static MapFusionPtr create(const std::string& pluginName) {
    SPtr<SharedLibrary> plugin = Registry::get(pluginName);
    if (!plugin) return MapFusionPtr();
    funcCreateMapFusionInstance createFunc =
        (funcCreateMapFusionInstance)plugin->getSymbol(
            "createMapFusionInstance");
    if (!createFunc)
      return MapFusionPtr();
    else
      return createFunc();
  }

 protected:
  GObjectHandle* cbk;
};
}  // namespace GSLAM

#endif  // GSLAM_CORE_MAPFUSION_H_
