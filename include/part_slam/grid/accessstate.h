//
// Created by zlc on 2021/5/25.
//

#ifndef _MY_SLAM_GMAPPING_ACCESSSTATE_H_
#define _MY_SLAM_GMAPPING_ACCESSSTATE_H_

namespace GMapping
{
    enum AccessibilityState
    {
        Outside=0x0,
        Inside=0x1,
        Allocated=0x2
    };
};


#endif // _MY_SLAM_GMAPPING_ACCESSSTATE_H_
