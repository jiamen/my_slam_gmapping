//
// Created by zlc on 2021/5/23.
//

#ifndef _MY_SLAM_GMAPPING_STAT_H_
#define _MY_SLAM_GMAPPING_STAT_H_


#include "../utils/point.h"
#include <vector>

namespace GMapping
{

double sampleGaussian(double sigma, unsigned int S=0);
double evalLoGaussian(double sigmaSquare, double delta);

// 协方差
struct Covariance3
{
    Covariance3 operator + (const Covariance3& cov) const;
    static Covariance3 zero;
    double xx, yy, tt, xy, xt, yt;
};


struct EigenCovariance3
{
    EigenCovariance3();
    EigenCovariance3(const Covariance3& c);
    EigenCovariance3 rotate(double angle) const;
    OrientedPoint sample() const;
    double eval[3];
    double evec[3][3];
};


struct Gaussian3
{
    OrientedPoint mean;                 // 均值
    EigenCovariance3 covariance;        // 协方差
    Covariance3 cov;
    double eval(const OrientedPoint& p) const;
};


};  // end namespace


#endif // _MY_SLAM_GMAPPING_STAT_H_
