//
// Created by zlc on 2021/5/23.
//
#include "../include/part_slam/motionmodel/motionmodel.h"

namespace GMapping
{

// 里程计运动模型
// 表示粒子估计的最优位置（激光雷达上一个时刻的最优位置）
// 表示里程计算出来的新的位置
// 表示里程计算出来的旧的位置（即上一个里程计的位置）

// 运动模型推算地图坐标系下的激光雷达位姿，并不神秘，就是在三个变化量基础上加上高斯噪声，在与之前激光雷达位姿结合就算出来了，就是简单的加法运算

OrientedPoint MotionModel::drawFromMotion(const OrientedPoint &p, const OrientedPoint &pnew,
                                          const OrientedPoint &pold) const
{
    double sxy = 0.3 * srr;       // srr我理解为两轮子里程计的方差

    // 计算出 pnew 相对于 pold 走了多少距离
    // 这里的距离表达是相对于车身坐标系来说的
    OrientedPoint delta = absoluteDifference(pnew, pold);

    // 初始化一个位姿点
    OrientedPoint noisy_point(delta);

    // 走过的X轴方向的距离加入噪声
    noisy_point.x += sampleGaussian(srr* fabs(delta.x) + str* fabs(delta.theta) + sxy* fabs(delta.y));
    // 走过的Y轴方向的距离加入噪声
    noisy_point.y += sampleGaussian(srr* fabs(delta.y) + str* fabs(delta.theta) + sxy* fabs(delta.x));
    // 走过的Z轴方向的距离加入噪声
    noisy_point.theta += sampleGaussian(stt* fabs(delta.theta) + srt* sqrt(delta.x*delta.x + delta.y*delta.y));

    // 限制角度的范围  fmod()函数是对浮点型数据进行取模运算，就是计算noisy_point.theta/2*M_PI的余数。 因为要把角度差限定在[−π π]之间。
    noisy_point.theta = fmod(noisy_point.theta, 2*M_PI);
    if (noisy_point.theta > M_PI)
        noisy_point.theta -= 2*M_PI;

    // 把加入了噪声的值 加到粒子估计的最优的位置上 即得到新的位置（根据运动模型推算出来的位置）
    return absoluteSum(p, noisy_point);
}


};
