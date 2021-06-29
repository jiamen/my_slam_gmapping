//
// Created by zlc on 2021/5/23.
//

#ifndef MY_SLAM_GMAPPING_MOTIONMODEL_H
#define MY_SLAM_GMAPPING_MOTIONMODEL_H

// #include "../utils/point.h"  // 下面的stat.h已经包含过了
#include "../utils/stat.h"
#include <iostream>


namespace GMapping
{

// 里程计运动模型
// 1.1 srr 线性运动造成的线性误差的方差           linear
// 1.2 srt 线性运动造成的角度误差的方差

// 2.1 str 旋转运动造成的线性误差的方差
// 2.2 stt 旋转运动造成的角度误差的方差            第二个t: theta


struct MotionModel
{
    // 给点当前坐标 这一次的里程计信息，上一次的里程计信息 计算出来新的位置
    OrientedPoint drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const;
    double srr, srt, str, stt;
    // srr(float, default: 0.1): 平移时里程误差作为平移函数(rho/rho)
    // srt(float, default: 0.2): 平移时的里程误差作为旋转函数 (rho/theta)
    // str(float, default: 0.1): 旋转时的里程误差作为平移函数 (theta/rho)
    // stt(float, default: 0.2): 旋转时的里程误差作为旋转函数 (theta/theta)
};

}


#endif //MY_SLAM_GMAPPING_MOTIONMODEL_H
