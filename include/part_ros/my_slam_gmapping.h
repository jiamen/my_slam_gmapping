//
// Created by zlc on 2021/5/22.
//

#ifndef _MY_SLAM_GMAPPING_MY_SLAM_GMAPPING_H_
#define _MY_SLAM_GMAPPING_MY_SLAM_GMAPPING_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"  // 传感器消息
#include "std_msgs/Float64.h"       // 标准信息
#include "nav_msgs/GetMap.h"        // 导航部分

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

#include "../include/part_slam/gridfastslam/gridslamprocessor.h"

#include <boost/thread.hpp>

/***********************************************************************************************************
  基于滤波器的SLAM算法-《gmapping算法的删减版》
  在原来gmapping源码的基础之上，对其进行了大刀阔斧的更改。
  （1）删除几乎所有不需要的代码，对代码的运行结构也进行了调整
  （2）对该代码进行详细中文注释，以及对核心代码进行更改
***********************************************************************************************************/

// 从smap的二维数组存储格式，转到一维数组，数组序号也需要转换到一维数组
#define MAP_IDX(sx, i, j)   ((sx) * (j) + (i))

#define GMAPPING_FREE       (0)
#define GMAPPING_UNKNOWN    (-1)
#define GMAPPING_OCC        (100)

class MySlamGMapping
{
public:
    MySlamGMapping();
    ~MySlamGMapping();

    void init();            // 构造函数调用的初始化函数
    void startLiveSlam();   // 开始实时SLAM
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);           // scan的回调函数，ros的核心函数

    bool mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);  // 订阅动态动图
    void publishLoop(double transform_publish_period);
    void publishTransform();                                                    // 发布map和odom的TF变换    里程计与地图的变换

private:
    bool initMapper(const sensor_msgs::LaserScan& scan);                        // 第一帧scan初始化
    bool addScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& gmap_pose);   // 对所有帧scan都有效
    bool getOdomPose(const sensor_msgs::LaserScan& scan);                       // 获得当前帧对应的里程计位姿
    bool updateMap(const sensor_msgs::LaserScan& scan);                         // 更新地图操作，主要是rviz的可视化操作部分

    ros::NodeHandle node_;              // MySlamGMapping对象的句柄
    ros::Publisher sst_;                // 发布话题map
    ros::Publisher sstm_;               // 发布话题map_metadata
    ros::ServiceServer ss_;             // 发布服务dynamic_map

    tf::TransformListener tf_;          // 以下三行组合使用
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

    tf::TransformBroadcaster* tfB_;     // tf变换的发布者，发布map与odom的TF变换



    GMapping::GridSlamProcessor* gsp_;  // GridSlamSLAM的对象



};



#endif // _MY_SLAM_GMAPPING_MY_SLAM_GMAPPING_H_
