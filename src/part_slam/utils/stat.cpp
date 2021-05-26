//
// Created by zlc on 2021/5/25.
//

#include <stdlib.h>

#include <math.h>
#include "../include/part_slam/utils/stat.h"


namespace GMapping
{

double pf_ran_gaussian(double sigma)
{
    double x1, x2, w;
    double r;

    do
    {
        do { r = drand48(); } while (r == 0.0);       // drand48() 返回服从均匀分布的·[0.0, 1.0) 之间的 double 型随机数。
        x1 = 2.0 * r - 1.0;
        do { r = drand48(); } while (r == 0.0);
        x2 = 2.0 * r - 1.0;
        w = x1*x1 + x2*x2;
    } while(w > 1.0 || w == 0.0);

    return (sigma * x2 * sqrt(-2.0*log(w)/w));
}

double sampleGaussian(double sigma, unsigned int S)
{
    if (S != 0)
        srand(S);
    if (sigma == 0)
        return 0;
    return pf_ran_gaussian(sigma);
}

double evalLogGaussian(double sigmaSquare, double delta)
{
    if (sigmaSquare <= 0)
        sigmaSquare = 1e-4;
    return -.5 * delta * delta / sigmaSquare - .5 * log(2*M_PI*sigmaSquare);
}

double Gaussian3::eval(const OrientedPoint& p) const
{
    OrientedPoint q = p - mean;
    q.theta = atan2(sin(p.theta-mean.theta), cos(p.theta-mean.theta));

    double v1,v2,v3;
    v1 = covariance.evec[0][0]*q.x+covariance.evec[1][0]*q.y+covariance.evec[2][0]*q.theta;
    v2 = covariance.evec[0][1]*q.x+covariance.evec[1][1]*q.y+covariance.evec[2][1]*q.theta;
    v3 = covariance.evec[0][2]*q.x+covariance.evec[1][2]*q.y+covariance.evec[2][2]*q.theta;

    return evalLogGaussian(covariance.eval[0], v1) + evalLogGaussian(covariance.eval[1], v2)
            + evalLogGaussian(covariance.eval[2], v3);
}

}

