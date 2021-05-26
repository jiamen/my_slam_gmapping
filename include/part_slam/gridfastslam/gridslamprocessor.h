//
// Created by zlc on 2021/5/22.
//

#ifndef _MY_SLAM_GMAPPING_GRIDSLAMPROCESSOR_H_
#define _MY_SLAM_GMAPPING_GRIDSLAMPROCESSOR_H_

#include <string>
#include <list>
#include <map>
#include <set>
#include <iomanip>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
#include <omp.h>

#include "../include/part_slam/particlefilter/particlefilter.h"
#include "../include/part_slam/utils/stat.h"
// #include "../include/part_slam/utils/point.h"    // 在stat.h中已经包含过了
#include "../include/part_slam/sensor_range/rangereading.h"
#include "../include/part_slam/scanmatcher/scanmatcher.h"
#include "../include/part_slam/motionmodel/motionmodel.h"

namespace GMapping
{

// 这个类实现一个GridFastSLAM算法，实现了一个RBPF, 每个粒子都拥有自己的地图和激光雷达位姿
/*
 * 工作流程如下：
 *  每当收到 里程计数据 和 激光雷达数据 之后，每个粒子的位姿根据运动模型来更新
 *
 *  根据运动模型更新得到的新的位置随后被用来初始化scan-slam算法
 *  scan-matcher 为每个粒子执行了一个局部优化算法
 *  scan-matcher 被用运动模型得到的位置来初始化，然后根据自己的地图来优化位置。
 * */

class GridSlamProcessor
{
public:
    //
    struct TNode
    {
        TNode(const OrientedPoint& pose, TNode* parent=0);
        ~TNode();

        OrientedPoint pose;
        const RangeReading* reading;
        TNode* parent;
    };

    // 用来定义一个节点数组，存储多条轨迹
    typedef std::vector<GridSlamProcessor::TNode> TNodeVector;

    // 粒子滤波器中的粒子结构体，每个粒子有自己的地图、位姿、权重、轨迹
    // 轨迹是按照时间顺序排列的，叶子节点 表示最近的节点
    struct Particle
    {
        Particle(const ScanMatcherMap& map);

    };
};

}






#endif // _MY_SLAM_GMAPPING_GRIDSLAMPROCESSOR_H_
