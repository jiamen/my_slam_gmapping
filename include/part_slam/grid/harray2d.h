//
// Created by zlc on 2021/5/25.
//

#ifndef _MY_SLAM_GMAPPING_HARRAY2D_H_
#define _MY_SLAM_GMAPPING_HARRAY2D_H_

#include <set>
// 必须说明的是set关联式容器,在set中每个元素的值都唯一，而且系统能根据元素的值自动进行排序
// map 和 set的插入删除效率 比 用其他序列容器高

#include "../utils/point.h"
#include "../utils/autoptr.h"
#include "array2d.h"


namespace GMapping
{

// 这里的cell在gmapping里面指的是PointAccumulator，  Storage为HierarchicalArray2D<PointAccumulator>
/*
	HierarchicalArray2D 类继承自 Array2D ，成员变量存在一个指向地图块的二级指针，Cell ** m_cells;
	所以，该类是一个二维patch数组，一个patch就是一个地图块，一个二级指针，指向一块地图
	一个patch包含2^m_patchMagnitude * 2^m_patchMagnitude个栅格
	也就是说地图实际上是分为两层的，第一层的元素为Patch（每一块地图块） 第二层的元素为cell（每一个栅格）

	patch的大小等级由 m_patchMagnitude决定
	patch的实际大小等于 m_patchSize = 1<<m_patchMagnitude = 2^m_patchMagnitude
	patch的实际大小表示一个patch里面包含有m_patchSize*m_patchSize的cell
	在gmapping源码中 m_patchMagnitude的默认大小为5 也就是说patch的默认大小为32*32
*/

// 左移的知识讲解：https://blog.csdn.net/weixin_42837024/article/details/98734787
// 左移几位就是在原数乘2的几次方

// HierarchicalArray2D 的一个对象就是一个 存储“地图补丁”的二维数组（栅格地图）

template <class Cell>
class HierarchicalArray2D : public Array2D< autoptr< Array2D<Cell> > >
{
public:
    typedef std::set< point<int>, pointcomparator<int> > PointSet;

protected:
    // new 一个patch的函数，const IntPoint& p 无用
    virtual Array2D<Cell>* createPatch(const IntPoint& p) const;

    PointSet m_activeArea;  // 存储地图中使用到的Cell的坐标
    int m_patchMagnitude;   // patch的大小等级
    int m_patchSize;		// patch的实际大小


public:
    // 构造函数
    HierarchicalArray2D(int x_size, int y_size, int patchMagnitude=5);
    HierarchicalArray2D(const HierarchicalArray2D& hg);

    // 重载=等号运算符
    HierarchicalArray2D& operator=(const HierarchicalArray2D& hg);

    // 析构函数
    virtual ~HierarchicalArray2D() {  }

    // 调整存储“地图补丁”的二维数组的大小
    void resize(int ix_min, int iy_min, int ix_max, int iy_max);
    // 输入“栅格”的 栅格坐标 做入口参数，返回“地图补丁”的“栅格坐标”（就是地图补丁的坐标），也就是一个栅格属于哪个地图补丁
    inline IntPoint patchIndexes(int x, int y) const;
    inline IntPoint patchIndexes(const IntPoint& p) const   { return patchIndexes(p.x, p.y); }

    // 输入“栅格”的 栅格坐标 做入口参数，返回该“地图补丁”的内存是否已经分配
    inline bool isAllocated(int x, int y) const;
    inline bool isAllocated(const IntPoint& p) const    { return isAllocated(p.x, p.y); }

    // 输入“栅格”的 栅格坐标 做入口参数，返回一个栅格对象
    inline Cell& cell(int x, int y);
    inline Cell& cell(const IntPoint& p) { return cell(p.x,p.y); }
    inline const Cell& cell(int x, int y) const;
    inline const Cell& cell(const IntPoint& p) const { return cell(p.x,p.y); }

    // 输入 “栅格” 栅格坐标做入口参数，返回“地图补丁”的状态
    inline AccessibilityState cellState(int x, int y) const ;
    inline AccessibilityState cellState(const IntPoint& p) const { return cellState(p.x,p.y); }

    // 设置地图的有效区域
    inline void setActiveArea(const PointSet&, bool patchCoords=false);
    // 给有效区域(被激光扫过的区域)分配内存
    inline void allocActiveArea();

    // 基本函数
    inline int getPatchSize() const {return m_patchMagnitude;}
    inline int getPatchMagnitude() const {return m_patchMagnitude;}
    const PointSet& getActiveArea() const {return m_activeArea; }
};

// 这里调用了Array2D的构造函数，为每块 “地图补丁” 申请了内存，存储的都是每一个patch（地图补丁），而非 地图栅格
// 这样的做法就是为了节省内存
template <class Cell>                       // 比如地图大小x_size=128， patchMagnitude=5     128/(2^5) = 4个地图块patch， Array2D初始化的不是地图栅格，而是patch
HierarchicalArray2D<Cell>::HierarchicalArray2D(int x_size, int y_size, int patchMagnitude)
        : Array2D< autoptr< Array2D<Cell> > >::Array2D((x_size>>patchMagnitude), (y_size>>patchMagnitude))
{
    m_patchMagnitude = patchMagnitude;      // 地图补丁的大小等级    5
    m_patchSize = 1<<m_patchMagnitude;      // 每块地图补丁的边大小，而非每块地图补丁中的栅格数目
}

// 拷贝构造函数
// 这里调用了Array2D的构造函数，为每块 “地图补丁” 申请了内存，存储的都是每一个patch（地图补丁），而非地图栅格
template <class Cell>
HierarchicalArray2D<Cell>::HierarchicalArray2D(const HierarchicalArray2D& hg)
        :Array2D<autoptr< Array2D<Cell> > >::Array2D((hg.m_xsize>>hg.m_patchMagnitude), (hg.m_ysize>>hg.m_patchMagnitude))  // added by cyrill: if you have a resize error, check this again
{
    this->m_xsize = hg.m_xsize;
    this->m_ysize = hg.m_ysize;
    this->m_cells = new autoptr< Array2D<Cell> >*[this->m_xsize];
    for (int x=0; x<this->m_xsize; x ++)
    {
        this->m_cells[x] = new autoptr< Array2D<Cell> >[this->m_ysize];
        for (int y=0; y<this->m_ysize; y ++)
            this->m_cells[x][y] = hg.m_cells[x][y];
    }
    this->m_patchMagnitude = hg.m_patchMagnitude;   // 地图补丁的大小等级
    this->m_patchSize = hg.m_patchSize;             // 每块地图补丁的边大小，而非每块地图补丁中的栅格数目
}

// HierarchicalArray2D重载的 = 运算符，释放原来的内存，重新按照参数大小申请新的 “地图补丁” ，然后把新值赋上
template <class Cell>
HierarchicalArray2D<Cell>& HierarchicalArray2D<Cell>::operator=(const HierarchicalArray2D& hg)
{
    if (this->m_xsize!=hg.m_xsize || this->m_ysize!=hg.m_ysize)
    {
        for (int i=0; i<this->m_xsize; i++)
            delete [] this->m_cells[i];
        delete [] this->m_cells;

        this->m_xsize=hg.m_xsize;
        this->m_ysize=hg.m_ysize;
        this->m_cells=new autoptr< Array2D<Cell> >*[this->m_xsize];
        for (int i=0; i<this->m_xsize; i++)
            this->m_cells[i]=new autoptr< Array2D<Cell> > [this->m_ysize];
    }

    for (int x=0; x<this->m_xsize; x++)
    {
        for (int y=0; y<this->m_ysize; y++)
        {
            this->m_cells[x][y]=hg.m_cells[x][y];
        }
    }

    m_activeArea.clear();
    m_patchMagnitude = hg.m_patchMagnitude;
    m_patchSize = hg.m_patchSize;
    return *this;
}

// 调整存储 “地图补丁” 的二维数组的大小，也就是个数变化
template <class Cell>
void HierarchicalArray2D<Cell>::resize(int x_min, int y_min, int x_max, int y_max)
{
    // 新地图补丁数组的尺寸
    int x_size = x_max - x_min;
    int y_size = y_max - y_min;
    // 为新的“地图补丁”二维数组申请内存
    autoptr< Array2D<Cell> > ** new_cells = new autoptr< Array2D<Cell> > *[x_size];
    for (int x=0; x<x_size; x ++)
    {
        new_cells[x] = new autoptr< Array2D<Cell> >[y_size];
        for (int y=0; y<y_size; y++)
        {
            new_cells[x][y] = autoptr< Array2D<Cell> >(0);  // 指向为NULL
        }
    }
    // x_min、y_min最小值为0，因为地图坐标没有负值的缘故
    int dx = x_min < 0 ? 0 : x_min;
    int dy = y_min < 0 ? 0 : y_min;
    // 此方法可以合理的遍历内存，不越界也不做无用的访问，因为只取原来的值
    int Dx = x_max<this->m_xsize ? x_max : this->m_xsize;
    int Dy = y_max<this->m_ysize ? y_max : this->m_ysize;
    // 把原来的地图中的栅格数据赋值到新的内存中，然后销毁原来的指针
    for (int x=dx; x<Dx; x ++)
    {
        for (int y=dy; y<Dy; y ++)
        {
            new_cells[x-x_min][y-y_min]=this->m_cells[x][y];
        }
        delete [] this->m_cells[x];     // 销毁每一列，从小向大销毁
    }
    delete [] this->m_cells;
    this->m_cells = new_cells;      // 新的二级指针指向老的二级指针
    this->m_xsize = x_size;         // 新的存储“地图补丁”的二维数组大小
    this->m_ysize = y_size;
}

// 输入“栅格”栅格坐标 做入口函数，返回“地图补丁”的栅格坐标（第一层）
template <class Cell>
IntPoint HierarchicalArray2D<Cell>::patchIndexes(int x, int y) const
{
    if (x>=0 && y>=0)
        return IntPoint(x>>m_patchMagnitude, y>>m_patchMagnitude);
    return IntPoint(-1, -1);
}

// 输入“地图补丁” 的 “栅格坐标”IntPoint new一个Array2D<Cell>对象，指定大小（2^m_patchMagnitude,2^m_patchMagnitude）
// 和输入无关
template <class Cell>
Array2D<Cell>* HierarchicalArray2D<Cell>::createPatch(const IntPoint& ) const
{
    return new Array2D<Cell>(1<<m_patchMagnitude, 1<<m_patchMagnitude);
}

// 输入“栅格” 栅格坐标做入口参数，返回该“地图补丁”的内存是否已经分配，1代表分配了
template <class Cell>
bool HierarchicalArray2D<Cell>::isAllocated(int x, int y) const
{
    IntPoint c = patchIndexes(x, y);
    autoptr< Array2D<Cell> >& ptr = this->m_cells[c.x][c.y];
    return (ptr != 0);
}

// 输入“栅格”栅格坐标做入口参数，返回一个栅格对象（该地图补丁起始位置的栅格对象）
template <class Cell>
Cell& HierarchicalArray2D<Cell>::cell(int x, int y)
{
    IntPoint c = patchIndexes(x, y);        // 得到栅格坐标 所在的 补丁坐标
    assert(this->isInside(c.x, c.y));       // “地图补丁”的“栅格坐标”是否在整个存储“地图补丁”的二维数组中
    if (!this->m_cells[c.x][c.y])           // 该指针存在有意义的指向
    {
        Array2D<Cell>* patch = createPatch(IntPoint(x,y));  // new一个patch（地图补丁）
        this->m_cells[c.x][c.y] = autoptr<Array2D<Cell>>(patch);
    }
    autoptr< Array2D<Cell> >& ptr = this->m_cells[c.x][c.y];
    return (*ptr).cell( IntPoint(x-(c.x<<m_patchMagnitude), y-(c.y<<m_patchMagnitude)) );
}
template <class Cell>
const Cell& HierarchicalArray2D<Cell>::cell(int x, int y) const
{
    assert(isAllocated(x,y));
    IntPoint c = patchIndexes(x, y);
    const autoptr< Array2D<Cell> >& ptr = this->m_cells[c.x][c.y];
    return (*ptr).cell( IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude)) );
}

// 输入 “栅格” 栅格坐标做入口参数，返回“地图补丁”的状态，Outside=0x0, Inside=0x1, Allocated=0x2
template <class Cell>
AccessibilityState  HierarchicalArray2D<Cell>::cellState(int x, int y) const
{
    if (this->isInside(patchIndexes(x, y)))     // 是否存在
    {
        if(isAllocated(x, y))                   // 是否分配
            return (AccessibilityState)((int)Inside | (int)Allocated);
        else
            return Inside;
    }
    return Outside;
}

/*
    设置地图的有效区域
    pathCoords指示当前的PointSet的坐标是否是对应的Patch坐标，如果不是则需要转换到patch坐标然后再插入
    对于HierarchicalArray2D来说，考虑的尺度都是patch。因此进行扩充地图或者分配内存的时候，每次也都是以patch为单位的。
    因此如果插入的点的坐标不是patch的坐标，则需要转换为对应的patch坐标，然后再插入。

    △△△△△注意：这里只是把点存储到对应的队列中，并没有进行内存的分配，真正的内存分配在后面的allocActiveArea()函数 △△△△△
*/
template <class Cell>
void HierarchicalArray2D<Cell>::setActiveArea(const typename HierarchicalArray2D<Cell>::PointSet& aa, bool patchCoords)
{
    m_activeArea.clear();
    for (PointSet::const_iterator it = aa.begin(); it != aa.end(); it ++)
    {
        IntPoint p;
        if (patchCoords)    // 是否是patch的坐标
            p = *it;        // 是的话不需要转换
        else
            p = patchIndexes(*it);      // 不是patch坐标需要进行转换
        m_activeArea.insert(p);
    }
}

/*
    给有效区域(被激光扫过的区域)分配内存
    给setActiveArea()函数插入的patch进行内存的分配。
    如果插入的区域没有分配内存，则进行分配。
    如果已经分配了内存则不再分配。
*/
template <class Cell>
void HierarchicalArray2D<Cell>::allocActiveArea()
{
    for (PointSet::const_iterator it=m_activeArea.begin(); it!=m_activeArea.end(); it ++)
    {
        const autoptr< Array2D<Cell> >& ptr = this->m_cells[it->x][it->y];
        Array2D<Cell>* patch = 0;
        // 如果对应的active没有被分配内存 则进行内存分配
        // 一个patch的内存没有分配的话，是没有内存存储栅格的，也就是没有array2D的对象的二级指针
        if (!ptr)
        {
            patch = createPatch(*it);
        }
        // 如果已经分配了还是赋原值
        else
        {
            patch = new Array2D<Cell>(*ptr);
        }
        this->m_cells[it->x][it->y] = autoptr< Array2D<Cell> >(patch);
    }
}


};


#endif // _MY_SLAM_GMAPPING_HARRAY2D_H_
