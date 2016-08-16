/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include <set>

#include <boost/thread.hpp>

#include "OSLAM.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include <gui/gl/Win3D.h>


namespace ORB_SLAM {

class MapPoint;
class KeyFrame;

class Map: public pi::gl::Draw_Opengl
{
public:
    Map();

    void Run(void);

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetCurrentCameraPose(cv::Mat Tcw);
    void SetReferenceKeyFrames(const std::vector<KeyFrame*> &vpKFs);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    void ClearUnusedMapPoint(double ts=0.0);
    void ClearUnusedKeyFrame(double ts=0.0);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    cv::Mat GetCameraPose();
    std::vector<KeyFrame*> GetReferenceKeyFrames();
    std::vector<MapPoint*> GetReferenceMapPoints();

    int MapPointsInMap();
    int KeyFramesInMap();

    void SetFlagAfterBA();
    bool isMapUpdated();
    void ResetUpdated();

    unsigned int GetMaxKFid();

    void clear();

    void Draw_Something();

protected:
    std::set<MapPoint*>     mspMapPoints;
    std::set<KeyFrame*>     mspKeyFrames;

    std::list<MapPoint*>    m_lstMapPoints_NeedDelete;
    std::list<KeyFrame*>    m_lstKeyFrames_NeedDelete;
    std::set<size_t>        delListKF, delListMP;
    size_t                  nDelKF, nDelMP;


    std::vector<MapPoint*>  mvpReferenceMapPoints;

    unsigned int mnMaxKFid;

    boost::mutex mMutexMap;
    bool mbMapUpdated;
    GLuint listName;
};

} //namespace ORB_SLAM

#endif // MAP_H
