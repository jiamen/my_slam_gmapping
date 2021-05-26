//
// Created by zlc on 2021/5/23.
//

#ifndef _MY_SLAM_GMAPPING_SCANMATCHER_H_
#define _MY_SLAM_GMAPPING_SCANMATCHER_H_

#include "../include/part_slam/grid/smmap.h"
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

    double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
    inline double score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
    inline unsigned int likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* reading) const;

    void computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);
    void registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);

    static const double nullLikelihood;

private:

protected:
    unsigned int m_laserBeams;                              // 激光束的数量
    double       m_laserAngles[LASER_MAXBEAMS];             // 各个激光束的角度
    IntPoint*    m_linePoints;

    // 定义一大堆参数以及其set和get函数
    PARAM_SET_GET(double, laserMaxRange, protected, public, public)         // 激光的最大测距范围
    PARAM_SET_GET(double, usableRange, protected, public, public)           // 使用的激光的最大范围
    PARAM_SET_GET(double, gaussianSigma, protected, public, public)
    PARAM_SET_GET(double, likelihoodSigma, protected, public, public)
    PARAM_SET_GET(int,    kernelSize, protected, public, public)
    PARAM_SET_GET(double, optAngularDelta, protected, public, public)       // 优化时的角度增量
    PARAM_SET_GET(double, optLinearDelta, protected, public, public)        // 优化时的长度增量
    PARAM_SET_GET(unsigned int, optRecursiveIterations, protected, public, public)	//优化时的迭代次数
    PARAM_SET_GET(unsigned int, likelihoodSkip, protected, public, public)
    PARAM_SET_GET(bool, generateMap, protected, public, public)
    PARAM_SET_GET(double, enlargeStep, protected, public, public)
    PARAM_SET_GET(double, fullnessThreshold, protected, public, public)				 // 被认为是占用的阈值
    PARAM_SET_GET(double, angularOdometryReliability, protected, public, public)	 // 里程计的角度可靠性
    PARAM_SET_GET(double, linearOdometryReliability, protected, public, public)		 // 里程计的长度可靠性
    PARAM_SET_GET(double, freeCellRatio, protected, public, public)					 // free 和 occupany 的阈值
    PARAM_SET_GET(unsigned int, initialBeamsSkip, protected, public, public)    // 去掉初始的几个激光束的数量

};


// 输入当前地图 和 当前经过运动模型更新的激光雷达位姿p，以及激光雷达数据
// 遍历若干激光束，求当前激光雷达位姿p，在当前地图的匹配得分，越匹配，位姿得分越高
// 匹配的方式就是找到，击中点和九宫格中的点的差距最小的点，差距越小，得分越高，越匹配，并且遍历若干束激光数据，计算累计得分
inline double ScanMatcher::score(const ScanMatcherMap &map, const OrientedPoint &p, const double *readings) const
{
    double s = 0;
    const double* angle = m_laserAngles + m_initialBeamsSkip;
    // lp表示此刻激光雷达坐标系在地图坐标系下的坐标
    OrientedPoint lp = p;

    // 如果激光击中了某个点，那么沿着激光方向的freeDelta距离的地方，要是空闲才可以
    unsigned int skip = 0;
    double freeDelta = map.getDelta()*m_freeCellRatio;      // 默认0.05×1.414

    // 枚举所有的激光束，有时为了提高计算速度，不需要对所有的激光数据进行计算
    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r ++, angle ++)
    {
        skip ++;
        skip = skip>m_likelihoodSkip ? 0 : skip;
        if (skip || *r>m_usableRange || *r==0.0)
            continue;

        // 被激光雷达击中的点 在地图坐标系中的坐标phit
        Point p_hit = lp;
        p_hit.x += *r * cos(lp.theta + *angle);
        p_hit.y += *r * sin(lp.theta + *angle);
        // 击中点在栅格地图中的栅格坐标
        IntPoint ip_hit = map.world2map(p_hit);

        // 假设p_hit是被激光击中的点，这样的话沿着激光方向的前面一个点必定的空闲的
        Point p_free = lp;
        p_free.x += (*r - freeDelta)*cos(lp.theta+*angle);
        p_free.y += (*r - freeDelta)*sin(lp.theta+*angle);

        // p_hit 和 p_free的栅格坐标的差距ip_free
        p_free = p_free - p_hit;
        IntPoint ip_free = map.world2map(p_free);

        // 在kernel_Size大小的擦黄口中搜索出最优最可能被这个激光束击中的点 kernel_Size默认为1
        // 这里形象化描述就是以击中点为中心的九宫格和相邻未击中点的九宫格
        bool found = false;
        Point bestMu(0., 0.);
        for (int xx=-m_kernelSize; xx<=m_kernelSize; xx ++)
        {
            for (int yy = -m_kernelSize; yy<=m_kernelSize; yy ++)
            {
                IntPoint pr = ip_hit + IntPoint(xx, yy);
                IntPoint pf = pr + ip_free;

                // 得到各自对应的Cell
                const PointAccumulator& cell  = map.cell(pr);
                const PointAccumulator& f_cell = map.cell(pr);

                // 这束激光要合法必须要满足cell是被占用的，而f_cell是空闲的
                // (double)cell 使用操作符重载，表示被占用概率

            }
        }
    }

}



};


#endif // _MY_SLAM_GMAPPING_SCANMATCHER_H_
