#ifndef PNC_MAP_CREATOR_STRAIGHT_H_
#define PNC_MAP_CREATOR_STRAIGHT_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorStraight : public PNCMapCreatorBase // 直道地图
    {
    public:
        PNCMapCreatorStraight();
        PNCMap creat_pnc_map() override; // 实现创建pnc_map的函数
    private:
        void init_pnc_map();                                                                            // 初始化pnc_map对象，设置地图长度、宽度等基本属性
        void draw_straight_x(const double &lenght,const double &plus_flag,const double &ratio = 1.0); // 沿x轴画直线
 // 绘制直道的X边界线，参数包括直道长度、偏移标志（正负表示左右偏移）和比例（控制边界线的密度）
        
    };
} // namespace Planning
#endif // PNC_MAP_CREATOR_STRAIGHT_H_