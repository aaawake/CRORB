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

#include "ViewerMono.h"
#include <pangolin/pangolin.h>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <mutex>

namespace ORB_SLAM2
{

ViewerMono::ViewerMono(System* pSystem):
    mpSystem(pSystem)
{
    
}

void ViewerMono::Run()
{

    pangolin::CreateWindowAndBind("Exchange",200,200);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuExchange("menu.Exchange",false,false);

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        pangolin::FinishFrame();

        if(menuExchange)
        {
            ExchangeOtherCar();
            mpSystem->Reset();
            menuExchange  = false;
        }
    }

    SetFinish();
}

void ViewerMono::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ViewerMono::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ViewerMono::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool ViewerMono::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void ViewerMono::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool ViewerMono::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool ViewerMono::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void ViewerMono::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void ViewerMono::ExchangeOtherCar()
{
    ros::NodeHandle nodeHandler;
    ros::Publisher pub = nodeHandler.advertise<std_msgs::UInt8>("Exchange", 1000); 

    std_msgs::UInt8 msg;
    msg.data = 1;

    pub.publish(msg);
    ROS_INFO("Exchange!");
}

}