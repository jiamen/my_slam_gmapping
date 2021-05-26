//
// Created by zlc on 2021/5/22.
//

#ifndef _MY_SLAM_GMAPPING_PARTICLEFILTER_H_
#define _MY_SLAM_GMAPPING_PARTICLEFILTER_H_

#include <stdlib.h>
#include <sys/types.h>
#include <vector>
#include <utility>
#include <cmath>
#include <limits>

template <class Particle, class Numeric>
struct uniform_resampler
{
    std::vector<unsigned int> resampleIndexes(const std::vector<Particle>& particles, int nparticles=0) const;
    std::vector<Particle> resample(const std::vector<Particle>& particles, int nparticles=0) const;
    Numeric neff(const std::vector<Particle>* particles) const;
};


//                 粒子            权重     本函数传入 粒子 和 粒子数量，返回权重比较大的粒子的索引
template <class Particle, class Numeric>
std::vector<unsigned int> uniform_resampler<Particle, Numeric>::resampleIndexes(const std::vector<Particle>& particles,
                                                                                int nparticles) const
{
    Numeric cweight = 0;    // 总权重

    // compute the cumulative weights
    /* 计算总的权重 */
    unsigned int n = 0;     // 统计实际粒子的数量
    for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); it ++)
    {
        cweight += (Numeric) * it;      // 权重=权值*粒子
        n ++;
    }

    if (nparticles > 0)
        n = particles;

    // compute the interval 计算间隔，其实就是权重的平均值
    Numeric interval = cweight/n;

    // compute the initial target weight  计算初始的目标权重
    Numeric target = interval * ::drand48();            // drand48()是用来产生{0,1}中任意随机数的，在linux系统下才有

    // compute the resampled indexes
    // 根据权值进行采样
    // 采用度盘轮转算法，target每次加interval。如果某个粒子的权重比较大的话，那么它肯定会被采样到很多次。
    // 比如说如果某个粒子的区间为4*interval。那么它至少被采样3次
    cweight = 0;
    std::vector<unsigned int> indexes(n);   // 存放重采样的粒子索引
    n = 0;
    unsigned int i = 0;
    for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); it ++, i ++)
    {
        cweight += (Numeric) * it;
        while (cweight > target)
        {
            indexes[n ++] = i;              // 第n个索引 存放 第i个粒子
            target += interval;
        }
    }

    return indexes;
}

template <class Particle, class Numeric>
std::vector<Particle> uniform_resampler<Particle, Numeric>::resample(const std::vector<Particle> &particles,
                                                                     int nparticles) const
{
    Numeric cweight = 0;

    // compute the cumulative weights    计算粒子总的权重
    unsigned int n = 0;
    for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); it ++)
    {
        cweight += (Numeric) * it;
        n ++;
    }

    if (nparticles)Numeric
        n = particles;

    // weight of the particles after resampling    重采样之后粒子的权重
    double uw = 1./n;

    // compute the interval     计算间隔
    Numeric interval = cweight / n;

    // compute the initial target weight
    Numeric target = cweight / n;
    // compute the resampled indexes

    cweight = 0;
    std::vector<Particle> resampled;        // 存放重采样的粒子
    n = 0;
    unsigned int i = 0;
    for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); it ++, i ++)
    {
        cweight += (Numeric) * it;
        while (cweight > target)
        {
            resampled.push_back(*it);
            resampled.back().setWeight(uw); // 对刚存入的粒子设置初始权重
            target += interval;
        }
    }

    return resampled;
}

template <class Particle, class Numeric>
Numeric uniform_resampler<Particle, Numeric>::neff(const std::vector<Particle> *particles) const
{
    double cum = 0;
    double sum = 0;
    for (typename std::vector<Particle>::const_iterator it=particles->begin(); it!=particles->end(); it ++)
    {
        Numeric w = (Numeric) * it;
        cum += w*w;
        sum += w;
    }

    return sum * sum / cum;         // 和的平方 / 平方和
}

#endif // _MY_SLAM_GMAPPING_PARTICLEFILTER_H_
