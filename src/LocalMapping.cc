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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"


#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbWrongInitResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{

    mbFinished = false;
    mbLightInit = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        // if(CheckNewKeyFrames())
        if(CheckAllNewKeyFrames())
        {
            // Set Robot Type by current keyframe
            SetRobotType();

            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            // if(!CheckNewKeyFrames())
            if(!CheckAllNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            // if(!CheckNewKeyFrames() && !stopRequested())
            if(!CheckAllNewKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustment2(mpCurrentKeyFrame,&mbAbortBA, mpMap);
                // if(mpMap->KeyFramesInMap()>2)
                // {
                //     if(mpCurrentLightKeyFrame)
                //     {
                //         Optimizer::LocalBundleAdjustment2(mpCurrentLightKeyFrame, mpCurrentKeyFrame, &mbAbortBA, mpMap, mbLightInit);
                //         mbLightInit = true;
                //     }
                //     else
                //         Optimizer::LocalBundleAdjustment2(mpCurrentKeyFrame,&mbAbortBA, mpMap);
                // }

                // Check redundant local Keyframes
                KeyFrameCulling();
            }
            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        WrongInitResetIfRequested();
        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}

void LocalMapping::InsertLightKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexLightKeyFrame);
    mlNewLightKeyFrames.push_back(pKF);
}

void LocalMapping::InsertMoveKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexMoveKeyFrame);
    mlNewMoveKeyFrames.push_back(pKF);
}

bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

bool LocalMapping::CheckMoveNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewMoveKeyFrames.empty());
}

bool LocalMapping::CheckLightNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewLightKeyFrames.empty());
}

bool LocalMapping::CheckAllNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty() || !mlNewMoveKeyFrames.empty() || !mlNewLightKeyFrames.empty());
}

void LocalMapping::SetRobotType()
{
    if(!mlNewMoveKeyFrames.empty())
        mnRobotType = 1;
    else if(!mlNewLightKeyFrames.empty())
        mnRobotType = 0;
    else
        mnRobotType = 2;
}

void LocalMapping::ProcessNewKeyFrame()
{
    if(mnRobotType == 1)
    {
        // std::cout << "mlNewMoveKeyFrames.size()=" << mlNewMoveKeyFrames.size() << endl;
        unique_lock<mutex> lock(mMutexMoveKeyFrame);
        mpCurrentKeyFrame = mlNewMoveKeyFrames.front();
        mlNewMoveKeyFrames.pop_front();

        // mpCurrentLightKeyFrame = mlNewLightKeyFrames.front();
        // mlNewLightKeyFrames.pop_front();
    }
    else if(mnRobotType == 0)
    {
        // std::cout << "mlNewLightKeyFrames.size()=" << mlNewMoveKeyFrames.size() << endl;
        unique_lock<mutex> lock(mMutexLightKeyFrame);
        mpCurrentKeyFrame = mlNewLightKeyFrames.front();
        mlNewLightKeyFrames.pop_front();
    }
    else if(mnRobotType == 2)
    {
        // std::cout << "mlNewKeyFrames.size()=" << mlNewMoveKeyFrames.size() << endl;
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
        mpCurrentLightKeyFrame = nullptr;
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    bool bCy=0;
    list<MapPoint*> lCyPoint;
    MapCylinder* mCy;
    cv::Mat CyDir;
    bool bmCy;
    float cosCyDirRays;
    bool bParallalCy;

    std::vector<cv::Mat> mvpxn1;
    mvpxn1.reserve(500);
    std::vector<cv::Mat> mvpxn2;
    mvpxn2.reserve(500);
    std::vector<cv::Mat> mvpTcw2;
    mvpTcw2.reserve(500);
    std::vector<KeyFrame*> mvpKFs;
    mvpKFs.reserve(500);
    std::vector<int> mvpIdx1;
    mvpIdx1.reserve(500);
    std::vector<int> mvpIdx2;
    mvpIdx2.reserve(500);


    if((mpMap->GetCurCylinder()!=static_cast<MapCylinder*>(NULL))){
        mCy=mpMap->GetCurCylinder();
        if(mCy->bActive==1) bCy=1;
    }
    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        // if(i>0 && CheckNewKeyFrames())
        if((i>0 && mnRobotType==1 && CheckMoveNewKeyFrames()) || 
           (i>0 && mnRobotType==0 && CheckLightNewKeyFrames()) ||
           (i>0 && mnRobotType==2 && CheckNewKeyFrames()))
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;
        if(bCy==1) CyDir=mCy->GetDirection();

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            cv::Mat x3D;
            bmCy=0;
            bParallalCy=0;

            if(bCy==1){
                cosCyDirRays = ray2.dot(CyDir)/(cv::norm(ray2)*cv::norm(CyDir));
                if(cosCyDirRays<0.99) {
                    cv::Mat Ow2=-Rwc2*tcw2;
                    bool bCyPoint=CreateCylindricalPoint(mCy, ray2, Ow2, x3D);
                    if(bCyPoint)
                    {
                        cv::Mat x3Dt = x3D.t();
                        //Check triangulation in front of cameras
                        float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
                    
                        // if(z1>0 && bCyPoint){
                        if(z1>0){
                            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
                            if(z2>0){
                                //Check reprojection error in first keyframe
                                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                                const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
                                const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
                                const float invz1 = 1.0/z1;

                                float u1 = fx1*x1*invz1+cx1;
                                float v1 = fy1*y1*invz1+cy1;
                                float errX1 = u1 - kp1.pt.x;
                                float errY1 = v1 - kp1.pt.y;
                                if((errX1*errX1+errY1*errY1)<5.991*sigmaSquare1) bmCy=1;
                            }
                        }
                    }
                }
                else bParallalCy=1;
            }
            if(bmCy==0)
            {
                const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

                float cosParallaxStereo = cosParallaxRays+1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

                if(bStereo1)
                    cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
                else if(bStereo2)
                    cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

                cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

                if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
                {
                    // Linear Triangulation Method
                    cv::Mat A(4,4,CV_32F);
                    A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                    A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                    A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                    A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                    cv::Mat w,u,vt;
                    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                    x3D = vt.row(3).t();

                    if(x3D.at<float>(3)==0)
                        continue;

                    // Euclidean coordinates
                    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
                }
                else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
                {
                    x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
                }
                else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
                {
                    x3D = pKF2->UnprojectStereo(idx2);
                }
                else
                    continue; //No stereo and very low parallax

                cv::Mat x3Dt = x3D.t();

                //Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
                if(z1<=0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
                if(z2<=0)
                    continue;

                //Check reprojection error in first keyframe
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
                const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
                const float invz1 = 1.0/z1;

                if(!bStereo1)
                {
                    float u1 = fx1*x1*invz1+cx1;
                    float v1 = fy1*y1*invz1+cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                        continue;
                }
                else
                {
                    float u1 = fx1*x1*invz1+cx1;
                    float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                    float v1 = fy1*y1*invz1+cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                        continue;
                }

                //Check reprojection error in second keyframe
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
                const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
                const float invz2 = 1.0/z2;
                if(!bStereo2)
                {
                    float u2 = fx2*x2*invz2+cx2;
                    float v2 = fy2*y2*invz2+cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                        continue;
                }
                else
                {
                    float u2 = fx2*x2*invz2+cx2;
                    float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                    float v2 = fy2*y2*invz2+cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                        continue;
                }

                //Check scale consistency
                cv::Mat normal1 = x3D-Ow1;
                float dist1 = cv::norm(normal1);

                cv::Mat normal2 = x3D-Ow2;
                float dist2 = cv::norm(normal2);

                if(dist1==0 || dist2==0)
                    continue;

                const float ratioDist = dist2/dist1;
                const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

                /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                    continue;*/
                if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                    continue;
            }
            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);
            pMP->AddObservation(mpCurrentKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
            if(bmCy==1) {
                lCyPoint.push_back(pMP);
                mvpxn1.push_back(xn1);
                mvpxn2.push_back(xn2);
                mvpTcw2.push_back(Tcw2);
                mvpKFs.push_back(pKF2);
                mvpIdx1.push_back(idx1);
                mvpIdx2.push_back(idx2);
            }
        }
    }
    if(float(lCyPoint.size())/nnew>0.65){
        mCy->CalculateLength(lCyPoint, -Rwc1*tcw1);
        mCy->AddCylindricalKF(mpCurrentKeyFrame);
    }
    else{
        if(mpMap->GetCandidateCylinder()==static_cast<MapCylinder*>(NULL)){
            MapCylinder* pCyNew = new MapCylinder(mpMap);
            mpMap->AddCandidateCylinder(pCyNew );
            mpMap->GetCandidateCylinder()->bActive=true;
            if(bCy==1) {
                mCy->bActive = false;
                if (mCy->mnBALocalForKF == 0) mCy->SetBadFlag();
            }
        }
        if(nnew==0 || lCyPoint.size()/nnew<0.5){
            size_t i=0;
            for(list<MapPoint*>::iterator lit=lCyPoint.begin(), lend=lCyPoint.end(); lit!=lend; lit++)
            {
                map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
                cv::Mat x3D;
                bool bx3D=0;
                bx3D = Triangulation(x3D, mvpxn1[i], mvpxn2[i], Tcw1, mvpTcw2[i], mvpKFs[i], mvpIdx1[i], mvpIdx2[i]);
                if(bx3D==1)  (*lit)->SetWorldPos(x3D);
                else (*lit)->SetBadFlag();
                i++;
            }
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

bool LocalMapping::Triangulation(cv::Mat& x3D, cv::Mat xn1, cv::Mat xn2, cv::Mat Tcw1, cv::Mat Tcw2, KeyFrame* pKF2, const int idx1, const int idx2){
    cv::Mat Rcw1 = Tcw1.colRange(0,3);
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = Tcw1.col(3);
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    cv::Mat Rcw2 = Tcw2.colRange(0,3);
    cv::Mat Rwc2 = Rcw2.t();
    cv::Mat tcw2 = Tcw2.col(3);
    cv::Mat Ow2 = pKF2->GetCameraCenter();
    const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float &fx2 = pKF2->fx;
    const float &fy2 = pKF2->fy;
    const float &cx2 = pKF2->cx;
    const float &cy2 = pKF2->cy;
    const float &invfx2 = pKF2->invfx;
    const float &invfy2 = pKF2->invfy;
    // Linear Triangulation Method
    cv::Mat A(4,4,CV_32F);
    A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
    A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
    A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
    A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

    cv::Mat w,u,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

    x3D = vt.row(3).t();

    if(x3D.at<float>(3)==0)
        return 0;

    // Euclidean coordinates
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

    cv::Mat x3Dt = x3D.t();
    //Check triangulation in front of cameras
    float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
    if(z1<=0)
        return 0;

    float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
    if(z2<=0)
        return 0;

    //Check reprojection error in first keyframe
    const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
    const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
    const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
    const float invz1 = 1.0/z1;

    float u1 = fx1*x1*invz1+cx1;
    float v1 = fy1*y1*invz1+cy1;
    float errX1 = u1 - kp1.pt.x;
    float errY1 = v1 - kp1.pt.y;
    if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
        return 0;


    //Check reprojection error in second keyframe
    const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
    const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
    const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
    const float invz2 = 1.0/z2;

    float u2 = fx2*x2*invz2+cx2;
    float v2 = fy2*y2*invz2+cy2;
    float errX2 = u2 - kp2.pt.x;
    float errY2 = v2 - kp2.pt.y;
    if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
        return 0;
    //Check scale consistency
    cv::Mat normal1 = x3D-Ow1;
    float dist1 = cv::norm(normal1);

    cv::Mat normal2 = x3D-Ow2;
    float dist2 = cv::norm(normal2);
    if(dist1==0 || dist2==0)
        return 0;

    const float ratioDist = dist2/dist1;
    const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];
    /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
        continue;*/
    if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
        return 0;
    return true;
}

bool LocalMapping::CreateCylindricalPoint(MapCylinder* Cy, cv::Mat ray, cv::Mat Pwc, cv::Mat &x3D){
    cv::Mat Para=Cy->GetWPara();
    cv::Mat Pcyw;
    cv::Mat Rcyw;
    cv::Mat Tcyw;
    Cy->GetCyCyMat(Rcyw,Pcyw,Tcyw);
    double r=Para.at<float>(4);
    cv::Mat Pcyc=Rcyw*Pwc+Pcyw;
    cv::Mat rcy=Rcyw*ray;

    double x=Pcyc.at<float>(0);
    double y=Pcyc.at<float>(1);
    double a=rcy.at<float>(0);
    double b=rcy.at<float>(1);
    double lambda=sqrt(pow((x*a+y*b),2)-(a*a+b*b)*(x*x+y*y-r*r))-(x*a+y*b);
    lambda/=(a*a+b*b);
    if(lambda>0){
        x3D=Pwc+ray*lambda;
        return true;
    }
    else return false;
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
//        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    mbLightInit = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    for(list<KeyFrame*>::iterator lit = mlNewLightKeyFrames.begin(), lend=mlNewLightKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewLightKeyFrames.clear();

    for(list<KeyFrame*>::iterator lit = mlNewMoveKeyFrames.begin(), lend=mlNewMoveKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewMoveKeyFrames.clear();

//    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::RequestWrongInitReset()
{
    {
        unique_lock<mutex> lock(mMutexWrongInitReset);
        mbWrongInitResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexWrongInitReset);
            if(!mbWrongInitResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        // mlNewKeyFrames.clear();
        // mlNewLightKeyFrames.clear();
        // mlNewMoveKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::WrongInitResetIfRequested()
{
    unique_lock<mutex> lock(mMutexWrongInitReset);
    if(mbWrongInitResetRequested)
    {
        mlNewKeyFrames.clear();
        mlNewLightKeyFrames.clear();
        mlNewMoveKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbWrongInitResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::setCurrentLightKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexLightKeyFrame);
    mpCurrentLightKeyFrame = pKF;
}
void LocalMapping::setCurrentMoveKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexMoveKeyFrame);
    mpCurrentMoveKeyFrame = pKF;
}

} //namespace ORB_SLAM
