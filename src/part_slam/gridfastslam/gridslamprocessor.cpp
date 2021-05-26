//
// Created by zlc on 2021/5/26.
//

#include "../include/part_slam/scanmatcher/scanmatcher.h"
#include "../include/part_slam/scanmatcher/gridlinetraversal.h"

#include <cstring>
#include <limits>
#include <list>
#include <iostream>
#include <omp.h>        // OpenMP , 多线程头文件
#include "ros/ros.h"
#include "tf/LinearMath/Scalar.h"


namespace GMapping
{

using namespace std;

const double ScanMatcher::nullLikelihood = -.5;

// 构造函数
ScanMatcher::ScanMatcher()
{
    m_laserBeams = 0;
    // 爬山算法减少搜索步长的次数
    m_optRecursiveIterations = 9;
    // 地图进行拓展的大小
    m_enlargeStep = 10.;
    // 判断栅格是否被占据的概率阈值
    m_fullnessThreshold = 0.1;

    // 指示里程计和陀螺仪是否可靠
    // 如果可靠的话，那么进行score计算的时候，就需要对离里程计数据比较远的位姿增加惩罚
    // 对于我们的应用来说，陀螺仪在短期内还是很可靠的
    m_angularOdometryReliability = 0.3;
    m_linearOdometryReliability  = 0.3;

    // 用来表示和击中栅格前面的栅格
    // 用来表示和击中栅格前面的栅格
    m_freeCellRatio=sqrt(2.);   // 1.414

    // 跳过一帧激光数据的开始几束激光
    m_initialBeamsSkip=0;

    m_linePoints = new IntPoint[20000];
}




};

