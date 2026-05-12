#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"

namespace Planning
{
    class GlobalPlannerNormal : public GlobalPlannerBase // 普通全局路径规划器
    {
    public:
        GlobalPlannerNormal();
        Path search_global_path(const PNCMap &pnc_map) override;
    private:
    public:
        void init_global_path(const PNCMap &pnc_map); // 初始化全局路径，输入为PNC地图对象，输出为规划的全局路径对象
    };
} // namespace Planning
#endif // GLOBAL_PLANNER_NORMAL_H_
