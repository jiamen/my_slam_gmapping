//
// Created by zlc on 2021/5/23.
//

#ifndef _MY_SLAM_GMAPPING_SCANMATCHER_H_
#define _MY_SLAM_GMAPPING_SCANMATCHER_H_

#include "../include/part_slam/grid/map.h"
#include "../include/part_slam/utils/macro_params.h"

#define LASER_MAXBEAMS 2048         // 激光雷达最大的激光束的数量

namespace GMapping
{

class ScanMatcher
{
public:
    ScanMatcher();
    ~ScanMatcher();

    void setLaserParameters(unsigned int beams, double* angles);
    void setMatchingParameters(double urange,  double range, double sigma, int kernel_size, double lopt, double aopt,
                               int iterations, double likelihoodSigma=1, unsigned int likelihoodSkip=0);

    // 1、爬山算法优化每个粒子的位姿    基于匹配得分进行优化
    double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
    // 计算得分：一个score用于优化调整粒子pose作为参考
    inline double score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
    // 计算似然得分：一个likelihoodAndScore用于确定优化后的每个粒子pose对应的权重更新作为参考
    inline void likelihoodAndScore(double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;

    // 分配内存，计算地图
    // void computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);
    void computeMap(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);

    void registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);

    static const double nullLikelihood;

private:

protected:
    unsigned int m_laserBeams;                              // 激光束的数量
    double       m_laserAngles[LASER_MAXBEAMS];             // 各个激光束的角度
    IntPoint*    m_linePoints;                              // 存放临时击中点

    // 定义一大堆参数以及其set和get函数
    PARAM_SET_GET(double,   laserMaxRange,              protected, public, public)      // 激光的最大测距范围
    PARAM_SET_GET(double,   usableRange,                protected, public, public)      // 使用的激光的最大范围
    PARAM_SET_GET(double,   gaussianSigma,              protected, public, public)
    PARAM_SET_GET(double,   likelihoodSigma,            protected, public, public)
    PARAM_SET_GET(int,      kernelSize,                 protected, public, public)
    PARAM_SET_GET(double,   optAngularDelta,            protected, public, public)      // 优化时的角度增量
    PARAM_SET_GET(double,   optLinearDelta,             protected, public, public)      // 优化时的长度增量
    PARAM_SET_GET(unsigned int, optRecursiveIterations, protected, public, public)	// 优化时的迭代次数
    PARAM_SET_GET(unsigned int, likelihoodSkip,    protected, public, public)      // 计算score过程中，为提高激光线束计算速度，这里进行跳跃，likelihoodSkip=1，则隔一束激光线计算一束激光线
    PARAM_SET_GET(bool,     generateMap,                protected, public, public)
    PARAM_SET_GET(double,   enlargeStep,                protected, public, public)      // 地图扩展大小，当有新的击中点时需要扩展地图
    PARAM_SET_GET(double,   fullnessThreshold,          protected, public, public)      // 被认为是占用的阈值
    PARAM_SET_GET(double,   angularOdometryReliability, protected, public, public)	    // 里程计的角度可靠性
    PARAM_SET_GET(double,   linearOdometryReliability,  protected, public, public)		// 里程计的长度可靠性
    PARAM_SET_GET(double,   freeCellRatio,              protected, public, public)		// free 和 occupany 的阈值   击中点旁边的空闲点的距离差
    PARAM_SET_GET(unsigned int, initialBeamsSkip,  protected, public, public)      // 去掉初始的几个激光束的数量

};


/*
 * scanMatcher 主要涉及以下两个评价函数：
 *    一个score用于优化调整粒子pose作为参考，
 *    一个likelihoodAndScore用于确定优化后的每个粒子pose对应的权重更新作为参考。
 *
 * 基本依据就是每帧激光的每个激光点预测距离 与 统计距离(预测点对应栅格单元的pose统计均值) 的高斯距离的统计。
 * */


// 输入当前地图 和 当前经过运动模型更新的激光雷达位姿p，以及激光雷达数据
// 遍历若干激光束，求当前激光雷达位姿p，在当前地图的匹配得分，越匹配，位姿得分越高
// 匹配的方式就是找到，击中点和九宫格中的点的差距最小的点，差距越小，得分越高，越匹配，并且遍历若干束激光数据，计算累计得分
inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const
{
    // 要返回的score分数
    double s = 0;
    // 获得当前帧激光数据的每束激光角度
    const double* angle = m_laserAngles + m_initialBeamsSkip;
    // lp表示 此刻激光雷达坐标系laser_link 在 地图坐标系map 下的坐标
    OrientedPoint lp = p;

    // 如果激光击中了某个点，那么沿着激光方向的freeDelta距离的地方，要是空闲才可以
    unsigned int skip = 0;
    double freeDelta = map.getDelta()*m_freeCellRatio;          // 默认0.05×1.414，表示斜着一个栅格的距离

    // 枚举所有的激光束，有时为了提高计算速度，不需要对所有的激光数据进行计算
    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r ++, angle ++)     //
    {
        skip ++;    // m_likelihoodSkip默认为0,这里若设置1,每两束激光计算1束
        skip = skip>m_likelihoodSkip ? 0 : skip;
        if (skip || *r>m_usableRange || *r==0.0)
            continue;

        // 被激光雷达击中的点 在地图坐标系中的坐标phit
        Point p_hit = lp;
        p_hit.x += *r * cos(lp.theta + *angle);     // p_hit.x是在map坐标系下的激光雷达坐标，*r*cos(lp.theta + *angle)是在laser_link激光坐标系下的击中点坐标，二者叠加得到击中点在map坐标系下的坐标
        p_hit.y += *r * sin(lp.theta + *angle);
        // 击中点在栅格地图中的栅格坐标：物理坐标转换为栅格坐标
        IntPoint ip_hit = map.world2map(p_hit);

        // 假设p_hit是被激光击中的点，这样的话沿着激光方向的前面一个点必定是空闲的
        Point p_free = lp;
        p_free.x += (*r - freeDelta)*cos(lp.theta+*angle);
        p_free.y += (*r - freeDelta)*sin(lp.theta+*angle);

        // p_hit 和 p_free的栅格坐标的差距ip_free
        p_free = p_free - p_hit;
        IntPoint ip_free = map.world2map(p_free);

        // 在kernel_Size大小的窗口中搜索出最优最可能被这个激光束击中的点 kernel_Size默认为1
        // 这里形象化描述就是以击中点为中心的九宫格和相邻未击中点的九宫格
        bool found = false;
        Point bestMu(0., 0.);
        for (int xx=-m_kernelSize; xx<=m_kernelSize; xx ++)     // -1 0 1
        {
            for (int yy = -m_kernelSize; yy<=m_kernelSize; yy ++)   // -1 0 1
            {
                IntPoint pr = ip_hit + IntPoint(xx, yy);        // 击中点 栅格坐标       IntPoint 坐标
                IntPoint pf = pr + ip_free;                     // 相邻未击中点 栅格坐标

                // 根据栅格坐标，得到各自对应的 Cell 栅格
                const PointAccumulator& cell   = map.cell(pr);  // 获得障碍物点占据值
                const PointAccumulator& f_cell = map.cell(pf);  // 获得非障碍物点空闲值

                // 这束激光要合法必须要满足cell是被占用的，而f_cell是空闲的
                // (double)cell 使用操作符重载，表示被占用概率
                if (((double)cell )> m_fullnessThreshold && ((double)f_cell )<m_fullnessThreshold)
                {
                    // 这里要理解一个事情，world2map会将物理坐标小于地图分辨率容差的物理坐标 分到一个栅格
                    // 所以才累计栅格被击中时的物理位置求平均，也就是cell.mean()，也就是历史栅格物理位姿的平均值
                    // 通过mu表示击中点和九宫格中的点的差距
                    // 我们寻找最小差距的那一个点
                    Point mu = p_hit - cell.mean();               // 重中之重 △△△ △△△

                    if (!found)
                    {
                        bestMu = mu;
                        found  = true;
                    }
                    else    // found==true
                    {
                        // 遍历九宫格中所有符合条件的，寻找两点差距最小的距离
                        bestMu = (mu*mu) < (bestMu*bestMu)? mu : bestMu;
                    }
                }
            }
        }

        // score 的计算公式 exp(-1.0/sigma*d^2)  这里的sigma表示方差
        // m_gaussianSigma 默认为0.05
        if (found)
        {
            // exp单增函数，bestMu越小，tmp_score越大，该点与地图越匹配
            double tmp_score = exp(-1.0/m_gaussianSigma*bestMu*bestMu); // 每一个找到bestMu的激光束计算得分
            s += tmp_score;
        }
    }

    return s;
}


// 和score函数几乎一致，相同的地方就不做注释了
inline void ScanMatcher::likelihoodAndScore(double& l, const ScanMatcherMap& map,
                                            const OrientedPoint& p, const double* readings) const
{
    using namespace std;
    l = 0;
    const double* angle = m_laserAngles;
    OrientedPoint lp = p;

    // 如果没有击中的时候的似然值 nullLikehood = -0.5
    double noHit = nullLikelihood / (m_likelihoodSigma);

    double freeDelta = map.getDelta() * m_freeCellRatio;

    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r ++, angle ++)
    {
        if (*r > m_usableRange || *r == 0.0)
            continue;

        Point p_hit = lp;
        p_hit.x += *r * cos(lp.theta+*angle);
        p_hit.y += *r * sin(lp.theta+*angle);
        IntPoint ip_hit = map.world2map(p_hit);

        Point p_free = lp;
        p_free.x += (*r - freeDelta)*cos(lp.theta+*angle);
        p_free.y += (*r - freeDelta)*sin(lp.theta+*angle);
        p_free = p_free - p_hit;
        IntPoint ip_free = map.world2map(p_free);

        bool found = false;
        Point bestMu(0., 0.);
        for (int xx=-m_kernelSize; xx<=m_kernelSize; xx ++)
        {
            for (int yy=-m_kernelSize; yy<=m_kernelSize; yy ++)
            {
                IntPoint pr = ip_hit + IntPoint(xx, yy);
                IntPoint pf = ip_hit + ip_free;
                const PointAccumulator& cell   = map.cell(pr);
                const PointAccumulator& f_cell = map.cell(pf);

                if (((double)cell )>m_fullnessThreshold && ((double)f_cell )<m_fullnessThreshold)
                {
                    Point mu=p_hit - cell.mean();
                    if (!found)
                    {
                        bestMu=mu;
                        found=true;
                    }
                    else
                    {
                        bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
                    }
                }
            }
        }

        // 似然不是指数 似然只是指数的上标，误差越小，似然越大，似然大小，代表权重大小
        double f = (-1. / m_likelihoodSigma) * (bestMu * bestMu);       //参数设置m_likelihoodSigma=0.075
        l += (found) ? f : noHit;
    }
}



};


#endif // _MY_SLAM_GMAPPING_SCANMATCHER_H_
