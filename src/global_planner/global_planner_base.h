#ifndef GLOBAL_PLANNER_BASE_H_
#define GLOBAL_PLANNER_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;
    enum class GlobalPlannerType
    {
        NORMAL // 普通全局路径规划器，基于A*算法实现
        // 后续可以添加更多全局路径规划器类型，如基于Dijkstra算法的全局路径规划器、基于RRT算法的全局路径规划器等
    };
    class GlobalPlannerBase // 全局路径规划器基类
    {
    public:
        virtual Path search_global_path(const PNCMap &pnc_map) = 0; // 规划全局路径的纯虚函数，具体实现由子类完成
        inline Path global_path() const { return global_path_; }    // 获取规划的全局路径对象
        virtual ~GlobalPlannerBase() {}                             // 虚析构函数，确保子类对象被正确销毁

    protected:
        std::unique_ptr<ConfigReader> global_planner_config_; // 配置读取器，子类可以使用它来读取全局路径规划相关的配置参数
        int global_planner_type_ = 0;                                // 全局路径规划器类型
        Path global_path_;                                    // 规划的全局路径对象，子类在规划全局路径时可以直接操作这个对象来生成规划结果
    };
} // namespace Planning
#endif // GLOBAL_PLANNER_BASE_H_
