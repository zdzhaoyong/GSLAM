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

#ifndef GSLAM_CORE_HASHMAP_H_
#define GSLAM_CORE_HASHMAP_H_
#include <string>
#include <unordered_map>
#include <utility>
#include "GSLAM.h"

namespace GSLAM {

typedef std::unordered_map<PointID, PointPtr> PointMap;
typedef std::unordered_map<FrameID, FramePtr> FrameMap;
typedef PointMap::iterator PointIt;
typedef FrameMap::iterator FrameIt;
typedef PointMap::const_iterator ConstPointIt;
typedef FrameMap::const_iterator ConstFrameIt;

class HashMap : public Map {
 public:
  HashMap() {}

  virtual ~HashMap() {}

  virtual std::string type() const { return "HashMap"; }

  /// MapFrame & MapPoint interface
  virtual bool insertMapPoint(const PointPtr& point);
  virtual bool insertMapFrame(const FramePtr& frame);
  virtual bool eraseMapPoint(const PointID& pointId);
  virtual bool eraseMapFrame(const FrameID& frameId);
  virtual void clear();

  virtual std::size_t frameNum() const;
  virtual std::size_t pointNum() const;

  virtual FramePtr getFrame(const FrameID& id) const;
  virtual PointPtr getPoint(const PointID& id) const;
  virtual bool getFrames(FrameArray& frames) const;
  virtual bool getPoints(PointArray& points) const;

  /// Save or load the map from/to the file
  virtual bool save(std::string path) const;
  virtual bool load(std::string path);

 private:
  PointMap _points;
  FrameMap _frames;
  mutable MutexRW _mutexPoints, _mutexFrames;
};

inline bool HashMap::insertMapPoint(const PointPtr& point) {
  WriteMutex lock(_mutexPoints);
  PointIt it = _points.find(point->id());
  if (it != _points.end()) return false;
  _points.insert(make_pair(point->id(), point));
  return true;
}

inline bool HashMap::insertMapFrame(const FramePtr& frame) {
  WriteMutex lock(_mutexFrames);
  FrameIt it = _frames.find(frame->id());
  if (it != _frames.end()) return false;
  _frames.insert(make_pair(frame->id(), frame));
  return true;
}

inline bool HashMap::eraseMapPoint(const PointID& pointId) {
  WriteMutex lock(_mutexPoints);
  return _points.erase(pointId) != 0;
}

inline bool HashMap::eraseMapFrame(const FrameID& frameId) {
  WriteMutex lock(_mutexFrames);
  return _frames.erase(frameId) != 0;
}

inline void HashMap::clear() {
  WriteMutex lock1(_mutexFrames);
  WriteMutex lock2(_mutexPoints);
  _points.clear();
  _frames.clear();
}

inline std::size_t HashMap::frameNum() const {
  ReadMutex lock(_mutexFrames);
  return _frames.size();
}

inline std::size_t HashMap::pointNum() const {
  ReadMutex lock(_mutexPoints);
  return _points.size();
}

inline FramePtr HashMap::getFrame(const FrameID& id) const {
  ReadMutex lock(_mutexFrames);
  ConstFrameIt it = _frames.find(id);
  if (it != _frames.end())
    return it->second;
  else
    return FramePtr();
}

inline PointPtr HashMap::getPoint(const PointID& id) const {
  ReadMutex lock(_mutexPoints);
  ConstPointIt it = _points.find(id);
  if (it != _points.end())
    return it->second;
  else
    return PointPtr();
}

inline bool HashMap::getFrames(FrameArray& frames) const {
  ReadMutex lock(_mutexFrames);
  frames.clear();
  frames.reserve(_frames.size());
  for (const std::pair<FrameID, FramePtr>& fr : _frames)
    frames.push_back(fr.second);
  return true;
}

inline bool HashMap::getPoints(PointArray& points) const {
  ReadMutex lock(_mutexPoints);
  points.clear();
  points.reserve(_points.size());
  for (const std::pair<PointID, PointPtr>& pt : _points)
    points.push_back(pt.second);
  return true;
}

/// Save or load the map from/to the file
inline bool HashMap::save(std::string path) const { return false; }

inline bool HashMap::load(std::string path) { return false; }
}  // namespace GSLAM
#endif  // GSLAM_CORE_HASHMAP_H_
