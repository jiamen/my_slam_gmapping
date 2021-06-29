//
// Created by zlc on 2021/6/24.
//

#ifndef _MY_SLAM_GMAPPING_LIDAR_UNDISTORTION_H_
#define _MY_SLAM_GMAPPING_LIDAR_UNDISTORTION_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include <iostream>
#include <dirent.h>
// dirent.h是用于目录操作的头文件，linux 默认在/usr/include目录下（会自动包含其他文件）,常见的方法如下：
//  1. opendir(): 打开目录，并返回句柄
//  2. readdir(): 读取句柄，返回dirent结构体
//  3. telldir(): 返回当前指针的位置，表示第几个元素
//  4. close(): 关闭句柄

#include <fstream>
#include <string>


/*******************************************************************************************************
 * 2D 激光雷达运动畸变去除
 * *****************************************************************************************************/

// 雷达运动畸变去除类
class LidarMotionCalibrator
{
public:
    // 构造函数
    LidarMotionCalibrator(std::string scan_frame_name, std::string odom_name);

    // 析构函数
    ~LidarMotionCalibrator();

    // 激光雷达运动畸变去除函数
    void lidarCalibration(std::vector<double>& ranges, std::vector<double>& angles,
                          ros::Time startTime, ros::Time endTime,
                          tf::TransformListener* tf_);

    // 从tf缓存数据中，寻找对应时间戳的里程计位姿
    bool getLaserPose(tf::Stamped<tf::Pose>& odom_pose, ros::Time dt, tf::TransformListener* tf_);

    // 根据传入参数，对任意一个分段进行插值
    void lidarMotionCalibration(tf::Stamped<tf::Pose> frame_base_pose, tf::Stamped<tf::Pose> frame_start_pose, tf::Stamped<tf::Pose> frame_end_pose,
                                std::vector<double>& ranges, std::vector<double>& angles,
                                int startIndex, int& beam_number);

public:
    // 针对各自的情况需要更改的名字，自行更改
    std::string  scan_frame_name_;          // 扫描帧 名字
    std::string  odom_name_;                // 里程计 名字
};




#endif // _MY_SLAM_GMAPPING_LIDAR_UNDISTORTION_H_
