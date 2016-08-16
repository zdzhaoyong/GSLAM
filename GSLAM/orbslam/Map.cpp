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

#include <base/Time/Global_Timer.h>
#include <base/Svar/Svar.h>

#include "Map.h"

using namespace pi;

namespace ORB_SLAM {

Map::Map()
{
    mbMapUpdated= false;
    mnMaxKFid = 0;
    listName=0;

    nDelKF = 0;
    nDelMP = 0;
}

void Map::Run(void)
{
    pi::Rate r(1);

    while( true ) {

        {
            boost::mutex::scoped_lock lock(mMutexMap);
            ClearUnusedMapPoint(pi::Timestamp().getTimestampF());
        }

        r.sleep();

        {
            boost::mutex::scoped_lock lock(mMutexMap);
            ClearUnusedKeyFrame(pi::Timestamp().getTimestampF());
        }

        /*
        printf("KF: %8d (deleted: %8d / total: %8d, %8d), MP: %8d (deleted: %8d / total: %8d, %8d)\n",
               mspKeyFrames.size(), nDelKF, KeyFrame::nNextId, KeyFrame::nNextId-nDelKF-mspKeyFrames.size(),
               mspMapPoints.size(), nDelMP, MapPoint::nNextId, MapPoint::nNextId-nDelMP-mspMapPoints.size());
        */

        r.sleep();
    }
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);

    mspKeyFrames.insert(pKF);
    if( pKF->mnId > mnMaxKFid )
        mnMaxKFid = pKF->mnId;

//    Win3D *win3d = SvarWithType<Win3D*>::instance()["MainWin3DPtr"];
//    cv::Mat center=pKF->GetCameraCenter();
//    win3d->setSceneCenter(qglviewer::Vec(center.at<float>(0),center.at<float>(1),center.at<float>(2)));
//    if(svar.GetInt("Win3D.AdaptedRadius",1))
//        win3d->setSceneRadius(win3d->camera()->distanceToSceneCenter()*4);

    mbMapUpdated = true;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);

    mspMapPoints.insert(pMP);
    mbMapUpdated = true;
}

// FIXME: need really delete the MapPoint
void Map::EraseMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);

    mspMapPoints.erase(pMP);
    mbMapUpdated = true;

    // add to after delete list
    pMP->setBadTS(pi::Timestamp::getTimestampF());
    m_lstMapPoints_NeedDelete.push_back(pMP);
}

// FIXME: need really delete the KeyFrame
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);

    mspKeyFrames.erase(pKF);
    mbMapUpdated = true;

    pKF->setBadTS(pi::Timestamp::getTimestampF());
    m_lstKeyFrames_NeedDelete.push_back(pKF);
}


// delete unused MapPoint
void Map::ClearUnusedMapPoint(double ts)
{
    if( !svar.GetInt("OSLAM.autoResouceClean", 1) ) return;

    double tsDelete = 10.0;
    size_t nOld = nDelMP;

    while( !m_lstMapPoints_NeedDelete.empty() ) {
        MapPoint *mp = m_lstMapPoints_NeedDelete.front();
        if( fabs(ts) < 1e-6 || ts - mp->getBadTS() > tsDelete ) {
            m_lstMapPoints_NeedDelete.pop_front();

            if( delListMP.end() == delListMP.find(mp->mnId) ) {
                delListMP.insert(mp->mnId);

                for(set<KeyFrame*>::iterator it=mspKeyFrames.begin();
                    it != mspKeyFrames.end(); it++) {
                    (*it)->EraseMapPoint(mp);
                }

                delete mp;
                nDelMP ++;
            } else {
//                printf("have deleted MP: %d", mp->mnId);
            }
        } else {
            break;
        }
    }

    //dbg_pt("delete %d MapPoints", nDelMP - nOld);
}

// delete unused KeyFrame
void Map::ClearUnusedKeyFrame(double ts)
{
    if( !svar.GetInt("OSLAM.autoResouceClean", 1) ) return;

    double tsDelete = 10.0;
    size_t nOld = nDelKF;

    while( !m_lstKeyFrames_NeedDelete.empty() ) {
        KeyFrame *kf = m_lstKeyFrames_NeedDelete.front();
        if( fabs(ts) < 1e-6 || ts - kf->getBadTS() > tsDelete ) {
            m_lstKeyFrames_NeedDelete.pop_front();

            if( delListKF.end() == delListKF.find(kf->mnId) ) {
                delListKF.insert(kf->mnId);

                for(set<MapPoint*>::iterator it=mspMapPoints.begin();
                    it != mspMapPoints.end(); it++) {
                    (*it)->EraseAssociatedKF(kf);
                }

                for(set<KeyFrame*>::iterator it=mspKeyFrames.begin();
                    it != mspKeyFrames.end(); it++) {
                    (*it)->EraseConnection(kf);

                    //(*it)->clearImage();
                }

                delete kf;
                nDelKF ++;
            } else {
//                ("have deleted KF: %d", kf->mnId);
            }
        } else {
            break;
        }
    }

    //dbg_pt("delete %d KeyFrames", nDelKF - nOld);
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    boost::mutex::scoped_lock lock(mMutexMap);

    mvpReferenceMapPoints = vpMPs;
    mbMapUpdated=true;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

int Map::MapPointsInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    return mspMapPoints.size();
}

int Map::KeyFramesInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    return mvpReferenceMapPoints;
}

bool Map::isMapUpdated()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    return mbMapUpdated;
}

void Map::SetFlagAfterBA()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    mbMapUpdated=true;

}

void Map::ResetUpdated()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    mbMapUpdated=false;
}

unsigned int Map::GetMaxKFid()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    return mnMaxKFid;
}

void Map::clear()
{
    boost::mutex::scoped_lock lock(mMutexMap);

    // delete unused MapPoint
    ClearUnusedMapPoint();

    // delete unused KeyFrame
    ClearUnusedKeyFrame();

    // reset delete list & counter
    nDelKF = 0;
    nDelMP = 0;
    m_lstKeyFrames_NeedDelete.clear();
    m_lstMapPoints_NeedDelete.clear();
    delListKF.clear();
    delListMP.clear();

    // free MapPoint and KeyFrame
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();

    // reset others
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
}

void Map::Draw_Something()
{
    timer.enter("Map::Draw_Something");
    glDisable(GL_LIGHTING);

    std::set<MapPoint*>     setMP;
    std::set<KeyFrame*>     setKF;

    {
        boost::mutex::scoped_lock lock(mMutexMap);

        setMP = mspMapPoints;
        setKF = mspKeyFrames;
    }

    if(!listName) listName=glGenLists(1);
    if( listName && setMP.size() > 0 )
    {
        if(mbMapUpdated)
        {
            timer.enter("Map::DrawData");

            //Draw MapDatas
            glNewList(listName,GL_COMPILE_AND_EXECUTE);
            {
                glColor3ub(255,255,255);
                glBegin(GL_POINTS);
                for(set<MapPoint*>::iterator it=setMP.begin();it!=setMP.end();it++)
                {
                    MapPoint& pt=**it;
                    if( pt.isBad() ) continue;
                    cv::Mat pose=pt.GetWorldPos();
                    glVertex3f(pose.at<float>(0),pose.at<float>(1),pose.at<float>(2));
                }
                glEnd();

                for(set<KeyFrame*>::iterator it=setKF.begin();it!=setKF.end();it++)
                {
                    KeyFrame& kf=**it;
                    if( kf.isBad() ) continue;
                    kf.draw();
                }
            }
            glEndList();
            mbMapUpdated=false;
            timer.leave("Map::DrawData");
        }
        else
        {
            timer.enter("Map::DrawList");
            glCallList(listName);
            timer.leave("Map::DrawList");
        }
    }

    timer.leave("Map::Draw_Something");
}

} //namespace ORB_SLAM
