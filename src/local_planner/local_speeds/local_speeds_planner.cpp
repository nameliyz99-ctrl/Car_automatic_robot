#include "local_speeds_planner.h"

namespace Planning
{
    LocalSpeedsPlanner::LocalSpeedsPlanner() // 速度规划器
    {
        // 1. 先初始化配置读取器
        local_speeds_config_ = std::make_unique<ConfigReader>();
        local_speeds_config_->read_local_speeds_config();

        // 2. 初始化平滑器（防止你在 cal_speed 里调用时崩溃）
        local_speeds_smoother_ = std::make_shared<LocalSpeedsSmoother>();
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local_speeds_planner created");
        // local_speeds_.header.frame_id = local_speeds_config_->pnc_map().frame_;
        // local_speeds_.header.stamp = rclcpp::Clock().now();
        // local_speeds_.local_speeds.clear(); // 根据决策中心的变道点位和速度点位，计算速度规划
        init_local_speeds();
    }

    LocalSpeeds LocalSpeedsPlanner::cal_speed(const std::shared_ptr<DecisionCenter> &decision)
    {

        // init_local_speeds();
        // double point_t = 0.0;
        // LocalSpeedsPoint point_tmp;

        // // 4. 安全检查：ST点不足无法规划
        // if (decision->st_points().size() < 2)
        // {
        //     RCLCPP_WARN(rclcpp::get_logger("local_speed"), "ST points not enough!");
        //     return local_speeds_;
        // }

        // // 5. 采样逻辑优化：增加采样密度（dt=0.1），改善 RViz 显示平滑度
        // const double dt = 0.1;
        // const int max_points = local_speeds_config_->local_speeds().speed_size_;

        // for (int i = 0; i < max_points; i++)
        // {
        //     double point_t = i * dt;
        //     LocalSpeedsPoint point_tmp;
        //     bool found = false;

        //     const int st_size = decision->st_points().size();
        //     // 6. 核心修复：边界必须是 j < st_size - 1
        //     for (int j = 0; j < st_size - 1; j++)
        //     {
        //         const auto &start_st = decision->st_points()[j];
        //         const auto &end_st = decision->st_points()[j + 1];

        //         if (point_t >= start_st.t_ && point_t < end_st.t_)
        //         {
        //             // 计算五次多项式 $s(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + a_4t^4 + a_5t^5$
        //             const Eigen::Vector<double, 6> a = PolynomialCurve::quintic_polynomial(
        //                 start_st.t_, end_st.t_,
        //                 start_st.s_2path_, end_st.s_2path_,
        //                 start_st.ds_dt_2path_, end_st.ds_dt_2path_,
        //                 0.0, 0.0); // 假定加速度起终点为 0

        //             double t2 = point_t * point_t;
        //             double t3 = t2 * point_t;
        //             double t4 = t3 * point_t;
        //             double t5 = t4 * point_t;

        //             point_tmp.t = point_t;
        //             point_tmp.s_2path = a[0] + a[1] * point_t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5;
        //             point_tmp.ds_dt_2path = a[1] + 2 * a[2] * point_t + 3 * a[3] * t2 + 4 * a[4] * t3 + 5 * a[5] * t4;
        //             point_tmp.dds_dt_2path = 2 * a[2] + 6 * a[3] * point_t + 12 * a[4] * t2 + 20 * a[5] * t3;

        //             found = true;
        //             break;
        //         }
        //     }

        //     if (found)
        //     {
        //         point_tmp.speed = point_tmp.ds_dt_2path;
        //         point_tmp.acceleration = point_tmp.dds_dt_2path;
        //         local_speeds_.local_speeds.emplace_back(point_tmp);
        //     }
        // }
        // local_speeds_smoother_->smooth_local_speeds(local_speeds_);
        // return local_speeds_;
        init_local_speeds();
        // 根据决策中心的变道点位和速度点位，计算速度规划
        // 这里简单地将决策中心的速度点位作为速度规划的点位，实际应用中需要根据决策中心的变道点位和速度点位，结合车辆的动力学模型，计算出合理的速度规划点位

        double point_t = 0.0;
        LocalSpeedsPoint point_tmp;
        for (int i = 0; i < local_speeds_config_->local_speeds().speed_size_; i++)
        {

            point_t += 1.0;
            if (point_t > local_speeds_config_->local_speeds().speed_size_)
            {
                break;
            }
            point_tmp.t = point_t;
            point_tmp.s_2path = local_speeds_config_->main_car().speed_ori_ * point_t; // 这里简单地将时间乘以一个系数作为距离，实际应用中需要根据车辆的动力学模型，计算出合理的距离
            point_tmp.ds_dt_2path = local_speeds_config_->main_car().speed_ori_;       // 这里简单地将时间乘以一个系数作为速度，实际应用中需要根据车辆的动力学模型，计算出合理的速度

            point_tmp.dds_dt_2path = 0.0; // 这里简单地将加速度设置为0，实际应用中需要根据车辆的动力学模型，计算出合理的加速度
            const int st_point_size = decision->st_points().size();
            for (int j = 0; j < st_point_size; j++)
            {
                const double start_t = decision->st_points()[j].t_;
                const double start_s = decision->st_points()[j].s_2path_;
                const double start_ds_dt = decision->st_points()[j].ds_dt_2path_;
                const double start_dds_dt = 0.0; // 这里简单地将加速度设置为0，实际应用中需要根据车辆的动力学模型，计算出合理的加速度

                const double end_t = decision->st_points()[j + 1].t_;
                const double end_s = decision->st_points()[j + 1].s_2path_;
                const double end_ds_dt = decision->st_points()[j + 1].ds_dt_2path_;
                const double end_dds_dt = 0.0; // 这里简单地将加速度设置为0，实际应用中需要根据车辆的动力学模型，计算出合理的加速度

                if (point_t >= start_t && point_t <= end_t)
                {
                    if (end_t == decision->st_points().back().t_ &&
                        end_s == decision->st_points().back().s_2path_)
                    {
                        const Eigen::Vector2d a = PolynomialCurve::linear_polynomial(start_t, start_s, end_t, end_s);
                        point_tmp.s_2path = a(0) + a(1) * point_t;
                        point_tmp.ds_dt_2path = a(1);
                        point_tmp.dds_dt_2path = 0.0; // 这里简单地将加速度设置为0，实际应用中需要根据车辆的动力学模型，计算出合理的加速度
                        if (fabs(point_tmp.s_2path) < min_speed)
                        {
                            point_tmp.ds_dt_2path = 0.0;
                        }
                    }

                    else
                    {
                        const double point_t_2 = point_t * point_t;
                        const double point_t_3 = point_t_2 * point_t;
                        const double point_t_4 = point_t_3 * point_t;
                        const double point_t_5 = point_t_4 * point_t;

                        const Eigen::Vector<double, 6> a = PolynomialCurve::quintic_polynomial(start_t, start_s, start_ds_dt,
                                                                                               start_dds_dt, end_t, end_s,
                                                                                               end_ds_dt, end_dds_dt);
                        point_tmp.s_2path = a(0) + a(1) * point_t + a(2) * point_t_2 + a(3) * point_t_3 + a(4) * point_t_4 + a(5) * point_t_5;
                        point_tmp.ds_dt_2path = a(1) + 2.0 * a(2) * point_t + 3.0 * a(3) * point_t_2 + 4.0 * a(4) * point_t_3 + 5.0 * a(5) * point_t_4;
                        point_tmp.dds_dt_2path = 2.0 * a(2) + 6.0 * a(3) * point_t + 12.0 * a(4) * point_t_2 + 20.0 * a(5) * point_t_3;
                    }
                }
            }

            point_tmp.speed = point_tmp.ds_dt_2path;         // 这里简单地将速度设置为ds_dt_2path，实际应用中需要根据车辆的动力学模型，计算出合理的速度
            point_tmp.acceleration = point_tmp.dds_dt_2path; // 这里简单地将加速度设置为dds_dt_2path，实际应用中需要根据车辆的动力学模型，计算出合理的加速度
                                                             // 这里简单地将加加速度设置为0，实际应用中需要根据车辆的动力学模型，计算出合理的加加速度
            local_speeds_.local_speeds.emplace_back(point_tmp);
        }
        local_speeds_smoother_->smooth_local_speeds(local_speeds_);
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "calculated local speeds,size: %ld", local_speeds_.local_speeds.size());
        return local_speeds_;
    }

    void LocalSpeedsPlanner::init_local_speeds()
    {

        local_speeds_.header.frame_id = local_speeds_config_->pnc_map().frame_;
        local_speeds_.header.stamp = rclcpp::Clock().now();
        local_speeds_.local_speeds.clear();
    }

} // namespace Planning