/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void InsertLightKeyFrame(KeyFrame* pKF);
    void InsertMoveKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestWrongInitReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }
    bool Triangulation(cv::Mat& x3D, cv::Mat xn1, cv::Mat xn2, cv::Mat Tcw1, cv::Mat Tcw2, KeyFrame* pKF2, const int idx1, const int idx2);
    bool CreateCylindricalPoint(MapCylinder* Cy, cv::Mat ray, cv::Mat Ow, cv::Mat &x3D);

    void setCurrentLightKeyFrame(KeyFrame* pKF);
    void setCurrentMoveKeyFrame(KeyFrame* pKF);
protected:

    bool CheckNewKeyFrames();
    bool CheckMoveNewKeyFrames();
    bool CheckLightNewKeyFrames();
    bool CheckAllNewKeyFrames();
    void SetRobotType();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    void WrongInitResetIfRequested();
    bool mbResetRequested;
    bool mbWrongInitResetRequested;
    std::mutex mMutexReset;
    std::mutex mMutexWrongInitReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;
    std::list<KeyFrame*> mlNewLightKeyFrames;
    std::list<KeyFrame*> mlNewMoveKeyFrames;

    KeyFrame* mpCurrentKeyFrame;
    KeyFrame* mpCurrentLightKeyFrame;
    KeyFrame* mpCurrentMoveKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    bool mbLightInit;
    bool mbNewMoveKeyFrame;
    std::mutex mMutexLightKeyFrame;
    std::mutex mMutexMoveKeyFrame;

    int mnRobotType;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
