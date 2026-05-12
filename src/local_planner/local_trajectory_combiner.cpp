#include "local_trajectory_combiner.h"

namespace Planning
{
    LocalTrajectoryCombiner::LocalTrajectoryCombiner() // 轨迹合成器
    {
        RCLCPP_INFO(rclcpp::get_logger("trajectory"), "local_trajectory_combiner created");
        trajectory_config_ = std::make_unique<ConfigReader>();

        // 读取配置文件
        //  轨迹合成器的配置文件主要包含局部轨迹的时间间隔、预测时间等参数，这些参数会影响局部轨迹的生成和更新频率，以及预测的时间范围。通过读取配置文件
    }

    LocalTrajectory LocalTrajectoryCombiner::combine_local_trajectory(const LocalPath &path, const LocalSpeeds &speeds)
    {
        const int path_size = path.local_path.size();
        const int speeds_size = speeds.local_speeds.size();
        local_trajectory_.header = path.header; // 局部轨迹的header与局部路径的header相同
        local_trajectory_.local_trajectory.clear();

        if (path_size < 3||speeds_size < 3)
        {
            RCLCPP_WARN(rclcpp::get_logger("trajectory"), "local path or local speeds is empty");
            return local_trajectory_;
        }
        LocalTrajectoryPoint point_tmp;
        for (int i = 0; i < path_size; i++)
        {
            // 路径部分填充
            point_tmp.path_point = path.local_path[i];
             if (i < speeds_size)
            {
                // 速度部分填充
           point_tmp.speed_point = speeds.local_speeds[i];
            }

            local_trajectory_.local_trajectory.emplace_back(point_tmp); // 将局部路径的点添加到局部轨迹中
        }

        RCLCPP_INFO(rclcpp::get_logger("trajectory"), "local trajectory combined, size: %ld", local_trajectory_.local_trajectory.size());
        return local_trajectory_;
    }

} // namespace Planning