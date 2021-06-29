//
// Created by zlc on 2021/5/26.
//

#include "../include/part_slam/gridfastslam/gridslamprocessor.h"


namespace GMapping
{

using namespace std;

// const double ScanMatcher::nullLikelihood = -.5;

// 单个节点的构造函数： 树的节点，一个树存了以整条轨迹，一个节点表示这条轨迹中的其中一个点。存储激光雷达的整条轨迹
GridSlamProcessor::TNode::TNode(const OrientedPoint& p, TNode* n)
{
    pose = p;
    parent = n;
    reading = 0;
}
GridSlamProcessor::TNode::~TNode()
{
    if (parent)
        delete parent;
}

// 单个粒子结构体的构造函数
GridSlamProcessor::Particle::Particle(const ScanMatcherMap& m) : map(m), pose(0, 0, 0), weight(0), weightSum(0)
{
    node = 0;
}

// GridSlamProcessor 主要类的 构造函数：初始化一些参数
GridSlamProcessor::GridSlamProcessor()
{
    period_             = 3.0;      // 两次更新时间的最小间隔
    m_obsSigmaGain      = 1;
    m_resampleThreshold = 0.5;      // 粒子选择性重采样的阈值
    m_minimumScore      = 0.;       // 认为匹配成功的最小得分阈值
}

GridSlamProcessor::~GridSlamProcessor()
{
    // 销毁所有粒子的轨迹
    for (std::vector<Particle>::iterator it=m_particles_.begin(); it!=m_particles_.end(); it ++)
    {
        if (it->node)
            delete it->node;
    }
}

void GridSlamProcessor::setMatchingParameters(double urange, double range, double sigma, int kernel_size,
                                              double l_opt,  double a_opt, int iterations, double likelihoodSigma,
                                              double likelihoodGain, unsigned int likelihoodSkip)
{
    m_obsSigmaGain = likelihoodGain;
    m_matcher.setMatchingParameters(urange, range, sigma, kernel_size, l_opt, a_opt, iterations, likelihoodSigma, likelihoodSkip);
}

// 设置运动模型参数
void GridSlamProcessor::setMotionModelParameters(double srr, double srt, double str, double stt)
{
    m_motionModel_.srr = srr;           // srr 线性运动造成的线性误差的方差
    m_motionModel_.srt = srt;           // srt 线性运动造成的角度误差的方差
    m_motionModel_.str = str;           // str 旋转运动造成的线性误差的方差
    m_motionModel_.stt = stt;           // stt 旋转运动造成的角度误差的方差
}


void GridSlamProcessor::setUpdateDistances(double linear, double angular, double resampleThreshold)
{
    m_linearThresholdDistance  = linear;
    m_angularThresholdDistance = angular;
    m_resampleThreshold = resampleThreshold;
}


void GridSlamProcessor::init(unsigned int size, double x_min, double y_min, double x_max, double y_max,
                             double delta, OrientedPoint initialPose)
{
    // 设置地图大小，分辨率
    m_x_min = x_min;
    m_y_min = y_min;
    m_x_max = x_max;
    m_y_max = y_max;
    m_delta = delta;

    // 初始化每个粒子
    m_particles_.clear();

    // new一个树的根节点，初始位姿initialPose，根节点为0
    TNode* node = new TNode(initialPose, 0);

    // 粒子对应的地图进行初始化
    ScanMatcherMap lmap(Point(x_min+x_max, y_min+y_max)*.5, x_max-x_min, y_max-y_min, delta);

    // 为每个粒子，设置地图，初始位姿，之前的位姿，权重等等
    for (unsigned int i=0; i<size; i ++)
    {
        m_particles_.push_back(Particle(lmap));     // 每个粒子设置的初始地图，同一个
        m_particles_.back().pose = initialPose;     // 每个粒子的初始位姿
        m_particles_.back().setWeight(0);        // 每个粒子的初始权重
        m_particles_.back().node = node;            // 每一个粒子设置初始父节点，同一个
    }

    m_neff = (double)size;
    m_count_ = 0;                                   // m_count表示这个函数被调用的次数
    m_linearDistance_ = m_angularDistance_ = 0;
}


// △△△△△△    SLAM核心代码    △△△△△△
bool GridSlamProcessor::processScan(const RangeReading& reading, int adaptParticles)
{
    // 得分当前激光雷达的里程计位姿
    OrientedPoint relPose = reading.getPose();

    // m_count表示这个函数被调用的次数
    if (!m_count_)      // 如果是第一次调用本函数
    {
        m_odoPose_ = relPose;       // 上一次的激光雷达里程计位姿 = 本次激光雷达里程计位姿
    }

    // 对于每一个粒子，都要通过里程计运动模型更新每一个粒子对象存储的地图坐标系下的激光雷达位姿
    int tmp_size = m_particles_.size();

    // 这里做一下说明，对于vector数组的遍历，对比了以下，release模式下几乎无差别，这里用最易读的方式
    for (int i=0; i<tmp_size; i ++)
    {
        // 对于每一个粒子，将从运动模型传播的激光雷达位姿存放到m_particles[i].pose （最新的地图坐标系下的激光雷达位姿）
        OrientedPoint& pose(m_particles_[i].pose);
        pose = m_motionModel_.drawFromMotion(m_particles_[i].pose, relPose, m_odoPose_);    // 这里一定要区分三个位姿
        // m_particles[i].pose:最新的地图坐标系下的激光雷达最优位姿  relPose:当前机关雷达里程计位姿   m_odoPose:表示上一次的激光雷达里程计位姿
    }

    /*根据两次里程计的数据 计算出来激光雷达的线性位移和角度变化*/
    OrientedPoint move = relPose - m_odoPose_;
    Point temp(move.x, move.y);
    // 激光雷达在里程计作用下的累计移动线性距离和累计角度变换，用于判断是否，进行核心算法处理
    // 处理完后，m_linearDistance 和 m_angularDistance  清零
    m_linearDistance_  += sqrt(temp*temp);          // 两点之间距离公式
    m_angularDistance_ += fabs(move.theta);

    // 更新上一次的激光雷达的里程计位姿
    m_odoPose_ = relPose;
    // 做返回值用
    bool processed = false;
    // 自己修改的，这个比较随意
    last_update_time_ += 1.0;

    // 只有当激光雷达走过一定的距离 或者 旋转过一定的角度  或者  经一段指定的时间才处理激光数据  或者 处理第一帧数据
    if (!m_count_
            || m_linearDistance_  >= m_linearThresholdDistance
            || m_angularDistance_ >= m_angularThresholdDistance
            || (period_ >= 0.0 && (last_update_time_) >period_))
    {
        last_update_time_ = 0;

        // 拷贝reading的scan的range数据
        int beam_number = reading.getSize();
        double* plainReading = new double[beam_number];
        for (unsigned int i=0; i<beam_number; i ++)
        {
            plainReading[i] = reading.m_dists[i];
        }

        // reading_copy数据用来放在每一个节点，构建地图用
        RangeReading* reading_copy = new RangeReading(beam_number, &(reading.m_dists[0]), &(reading.m_angles[0]));


        // 如果不是第一帧数据
        if (m_count_ > 0)
        {
            // 功能：为每一个粒子，在当前激光雷达位姿下，当前帧激光数据下，求解每个粒子的位姿最优值（策略：爬山，评价机制：得分机制（激光雷达在当前里程计位姿下激光数据和当前地图的的匹配程度））
            //      求解每个粒子的权重
            // 爬山算法：每一层，从不同方向寻找一个得分最高的一个值，直到出现得分开始下降的地方
            // 得分算法：根据当前位姿下的一帧激光扫描点和地图的匹配程度，对每一个激光点做计算，累积得分，返回得分

            // 通过扫描匹配寻找每一个粒子的激光雷达在地图坐标系下的最优位姿，并且计算粒子得分
            scanMatch(plainReading);

            // 计算重采样的neff（粒子离散程度）的判断值
            normalize();

            // 重采样
            if(resample(plainReading, adaptParticles, reading_copy))
            {
                //进行重采样之后，粒子的权重又会发生变化，更新归一化的权重，否则无需normalize
                normalize();
            }
        }
        else
        {
            // 如果是第一帧数据，则可以直接计算activeArea，因此这个时候，对激光雷达的位置是非常确定的，就是（0，0，0）
            for (ParticleVector::iterator it=m_particles_.begin(); it!=m_particles_.end(); it ++)
            {
                // 拓展地图大小、找到地图的有效区域，单位patch,申请内存、更新每个栅格的内容
                m_matcher.computeMap(it->map, it->pose, plainReading);

                // 为每个粒子创建路径的第一个节点。该节点的权重为0,父节点为it->node(这个时候为NULL)。
                TNode* node = new TNode(it->pose, it->node);
                node->reading = reading_copy;    //根节点的激光雷达数据
                it->node = node;
            }
            // 第一帧数据权重归一化更新
            normalize();
        }
        delete [] plainReading;

        // 激光雷达累计行走的多远的路程没有进行里程计的更新 每次更新完毕之后都要把这个数值清零
        m_linearDistance_  = 0;
        m_angularDistance_ = 0;

        // 对激光帧计数
        m_count_ ++;
        processed = true;
    }

    return processed;
}


// 找到最大权重粒子的序号
int GridSlamProcessor::getBestParticleIndex() const
{
    unsigned int bi=0;
    double bw = -std::numeric_limits<double>::max();

    for (unsigned int i=0; i<m_particles_.size(); i ++)
    {
        if (bw < m_particles_[i].weightSum)
        {
            bw = m_particles_[i].weightSum; // 粒子累计最大权重
            bi = i;                         // 粒子序号
        }
    }

    return (int) bi;
}


};

