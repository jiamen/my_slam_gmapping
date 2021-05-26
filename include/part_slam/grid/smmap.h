//
// Created by zlc on 2021/5/25.
//

#ifndef _MY_SLAM_GMAPPING_SMMAP_H_
#define _MY_SLAM_GMAPPING_SMMAP_H_

#include "map.h"
#include "harray2d.h"
#include "../utils/point.h"

#define SIGHT_INC 1

namespace GMapping
{

// PointAccumulator表示地图中一个cell（栅格）包括的内容
/*
 * acc: 栅格累计被击中位置
 * n:   栅格被击中次数
 * visits: 栅格被访问的次数
 * */

// PointAccumulator的一个对象，就是一个栅格，gmapping中其他类模板的cell就是这个

struct PointAccumulator
{
    // float 类型的 point
    typedef point<float> FloatPoint;
    // 该栅格被击中的位置累计，最后取累计值的均值
    FloatPoint acc;
    // n表示该栅格被击中的次数，visits表示该栅格被访问的次数
    int n, visits;
    // 构造函数
    PointAccumulator() : acc(0,0), n(0), visits(0)  {  }
    PointAccumulator(int i) : acc(0, 0), n(0), visits(0)    {  }
    // 计算栅格被击中坐标累计值的平均值
    inline Point mean() const { return 1./n*Point(acc.x, acc.y); }
    // 计算栅格被击中被占用的概率，范围是 -1 （没有访问过）、[0,1]
    inline operator double() const { return visits ? (double)n*SIGHT_INC/(double)visits : -1; }

    // 相同类型的栅格对象“+”操作
    inline void add(const PointAccumulator& p)  { acc=acc+p.acc; n+=p.n; visits+=p.visits; }
    // 更新该栅格成员变量
    inline void update(bool value, const Point& p=Point(0,0));
    // 计算栅格的信息熵
    inline double entropy() const;
    // 实例化unknown_ptr指针
    static const PointAccumulator& Unknown();
    // 静态指针，表示所有栅格的默认状态， =====待验证=====
    static PointAccumulator* unknown_ptr;
};

// 更新该栅格成员变量，value表示该栅格是否被击中，击中n++，未击中仅visits++；
void PointAccumulator::update(bool value, const Point& p)
{
    if (value)
    {
        acc.x += static_cast<float>(p.x);
        acc.y += static_cast<float>(p.y);
        n ++;
        visits += SIGHT_INC;
    }
    else
        visits ++;
}

// 计算栅格的信息熵
double PointAccumulator::entropy() const
{
    if (!visits)
        return -log(.5);    // 如果该栅格visits==0
    if (n==visits || n==0)
        return 0;             // 如果该栅格被击中等于访问次数，或者击中次数为0，则返回0

    double x = (double)n*SIGHT_INC/(double)visits;  // 该栅格被击中概率x
    return -( x* log(x) + (1-x)* log(1-x) );      // 信息熵计算公式 -(p*log(p)+(1-p)*log(1-p))，x只有两种可能，击中和未被击中
}


// 最终的地图类
typedef Map< PointAccumulator, HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;  // 一张地图的类

};


#endif // _MY_SLAM_GMAPPING_SMMAP_H_
