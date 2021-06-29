//
// Created by zlc on 2021/5/23.
//

#ifndef _MY_SLAM_GMAPPING_RANGEREADING_H_
#define _MY_SLAM_GMAPPING_RANGEREADING_H_


#include <vector>
#include "../utils/point.h"

namespace GMapping
{

// 激光传感器数据类
class RangeReading
{
public:
    // 均匀角度的激光雷达数据
    // RangeReading(unsigned int n_beams, const double* d);

    // 不均匀角度的激光雷达数据
    RangeReading(unsigned int n_beams, const double* d, const double* angle);

    virtual ~RangeReading();

    // 得到这帧激光数据的激光雷达位姿
    inline const OrientedPoint& getPose() const
    {
        return m_pose;
    }
    // 设置这帧传感器数据的位姿
    inline void setPose(const OrientedPoint& pose)
    {
        m_pose = pose;
    }
    // 返回激光束的多少
    inline const unsigned int getSize() const
    {
        return m_beams;
    }

    // 存储激光雷达的距离信息
    std::vector<double> m_dists;
    unsigned int        m_beams;

    // 每个激光束对应的角度
    std::vector<double> m_angles;

protected:
    // 这帧激光数据的位姿
    OrientedPoint m_pose;
};

}


#endif // _MY_SLAM_GMAPPING_RANGEREADING_H_
