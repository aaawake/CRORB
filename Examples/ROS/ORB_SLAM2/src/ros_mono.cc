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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <condition_variable>
#include <mutex>

#include<ros/ros.h>
#include <std_msgs/UInt8.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include "./Mono/ViewerMono.h"

using namespace std;

class ImageGrabber
{
public:
    std::mutex exchange_mutex;
    std::condition_variable exchange_cv;
    bool exchange_ready = true;
    ros::Publisher exchange_pub;

    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle& nh):mpSLAM(pSLAM){
        exchange_pub = nh.advertise<std_msgs::UInt8>("/Exchange", 1000);
    }

    void ExchangeCallback(const std_msgs::UInt8::ConstPtr& msg);
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabImage(const sensor_msgs::CompressedImageConstPtr& msg, const int robotID = 1);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    ros::Duration(6).sleep();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ORB_SLAM2::ViewerMono viewerMono(&SLAM);

    ros::NodeHandle nodeHandler;

    ImageGrabber igb(&SLAM, nodeHandler);
    // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::Subscriber exchange_sub = nodeHandler.subscribe<std_msgs::UInt8>(
        "/Exchange", 
        1, 
        [&igb](const std_msgs::UInt8ConstPtr& msg) { igb.ExchangeCallback(msg); }
    );

    // Move robot
    ros::Subscriber sub = nodeHandler.subscribe<sensor_msgs::CompressedImage>(
        "/robot_1/image_raw/compressed", 
        1, 
        [&igb](const sensor_msgs::CompressedImageConstPtr& msg) { igb.GrabImage(msg, 1); }
    );

    // Light robot
    ros::Subscriber light_sub = nodeHandler.subscribe<sensor_msgs::CompressedImage>(
        // 修改：改rostopic
        "/robot_0/image_raw/compressed", 
        1, 
        [&igb](const sensor_msgs::CompressedImageConstPtr& msg) { igb.GrabImage(msg, 0); }
    );

    // ros::Duration(1).sleep();
    // std_msgs::UInt8 initial_msg;
    // initial_msg.data = 0;
    // igb.exchange_pub.publish(initial_msg);
    // ROS_INFO("Initial Exchange message published with data=0.");

    ros::Duration(6).sleep();
    thread tViewer = thread(&ORB_SLAM2::ViewerMono::Run,&viewerMono);
    
    ros::MultiThreadedSpinner spinner(4); // 使用4个线程
    spinner.spin();
    // ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::ExchangeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    ROS_INFO("Received message: %d", msg->data);
    
    if (msg->data == 0)
    {
        std::lock_guard<std::mutex> lock(exchange_mutex);
        exchange_ready = true;
        exchange_cv.notify_all();
    }
    else
    {
        std::lock_guard<std::mutex> lock(exchange_mutex);
        exchange_ready = false;
        ROS_INFO("Waiting for Exchange...");
    }
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}

void ImageGrabber::GrabImage(const sensor_msgs::CompressedImageConstPtr& msg, const int robotID)
{
    {
        std::unique_lock<std::mutex> lock(exchange_mutex);
        while (!exchange_ready)
        {
            exchange_cv.wait(lock);
        }
    }

    // Copy the ros image message to cv::Mat.
    cv::Mat image;
    try
    {
        // 解码图像
        std::vector<uchar> data(msg->data.begin(), msg->data.end());
        image = cv::imdecode(data, cv::IMREAD_COLOR); // 指定读取为彩色图像

        if (image.empty())
        {
            ROS_ERROR("Failed to decode image");
            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->TrackMonocular(image,msg->header.stamp.toSec(), robotID);
}

