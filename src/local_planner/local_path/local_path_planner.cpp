#include "local_path_planner.h"

namespace Planning
{
    LocalPathPlanner::LocalPathPlanner()
    { // 局部路径规划器
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local_path_planner created");

        local_path_config_ = std::make_unique<ConfigReader>();
        local_path_config_->read_local_path_config();

        local_path_smoother_ = std::shared_ptr<LocalPathSmoother>();
    }
    LocalPath LocalPathPlanner::creat_local_path(const Referline &reference_line,
                                                 const std::shared_ptr<VehicleBase> &car,
                                                 const std::shared_ptr<DecisionCenter> &decision)
    {
        // init_local_path();
        // // 1. 获取起始 s 坐标
        // double start_s = car->to_path_frenet_params().s;
        // // 计算sl
        // double point_s = car->to_path_frenet_params().s;
        // double ds = 1.0; // s的步长，单位m，todo: johan，后续可以根据车速调整步长
        // LocalPathPoint point_tmp;
        // for (int i = 0; i < local_path_config_->local_path().path_size_; i++)
        // {
        //     // 3. 强制 s 严格递增
        //     double current_point_s = start_s + (i * step_size);
        //     // 规划起点
        //     point_s += car->to_path_frenet_params().ds_dt;             // 路径规划起点：下一帧参考线投影点的s m/帧
        //     if (current_point_s > reference_line.refer_line.back().rs) // 投影点超过参考线最前端，退出
        //     {
        //         break;
        //     }
        //     // 给point_tmp赋值，先给s ds_dt赋上，l和dl_ds初始化
        //     LocalPathPoint point_tmp;
        //     point_tmp.s = current_point_s;
        //     point_tmp.s = point_s;
        //     point_tmp.ds_dt = car->to_path_frenet_params().ds_dt;
        //     point_tmp.dds_dt = car->to_path_frenet_params().dds_dt;
        //     point_tmp.l = 0.0;
        //     point_tmp.dl_ds = 0.0;
        //     point_tmp.ddl_ds = 0.0;

        //     // 计算point_tmp的l和dl_ds
        //     const int sl_points_size = decision->sl_points().size();
        //     for (int j = 0; j < sl_points_size - 1; j++)
        //     {
        //         // 确定分段起始状态和末状态
        //         const double start_s = decision->sl_points()[j].s_;
        //         const double start_l = decision->sl_points()[j].l_;
        //         const double start_dl_ds = 0.0;
        //         const double start_ddl_ds = 0.0;

        //         const double end_s = decision->sl_points()[j + 1].s_;
        //         const double end_l = decision->sl_points()[j + 1].l_;
        //         const double end_dl_ds = 0.0;
        //         const double end_ddl_ds = 0.0;

        //         // 如果临时点在s的分段范围内
        //         if (point_s >= start_s && point_s < end_s)
        //         {
        //             // 五次多项式
        //             const double point_s_2 = point_s * point_s;
        //             const double point_s_3 = point_s_2 * point_s;
        //             const double point_s_4 = point_s_3 * point_s;
        //             const double point_s_5 = point_s_4 * point_s;
        //             const Eigen::Vector<double, 6> a = PolynomialCurve::quintic_polynomial(start_s, start_l, start_dl_ds, start_ddl_ds,
        //                                                                                    end_s, end_l, end_dl_ds, end_ddl_ds);
        //             point_tmp.l = a(0) + a(1) * point_s + a(2) * point_s_2 + a(3) * point_s_3 + a(4) * point_s_4 + a(5) * point_s_5;
        //             point_tmp.dl_ds = a(1) + 2.0 * a(2) * point_s + 3.0 * a(3) * point_s_2 + 4.0 * a(4) * point_s_3 + 5.0 * a(5) * point_s_4;
        //             point_tmp.ddl_ds = a(2) + 6.0 * a(3) * point_s + 12.0 * a(4) * point_s_2 + 20.0 * a(5) * point_s_3;
        //         }
        //     }

        //     local_path_.local_path.emplace_back(point_tmp);
        //
        init_local_path();

        // 1. 获取起始 s 坐标
        double start_s = car->to_path_frenet_params().s;

        // 2. 定义固定步长（建议 0.5m 或 1.0m，不要依赖 ds_dt）
        const double step_size = 1.0;

        for (int i = 0; i < local_path_config_->local_path().path_size_; i++)
        {
            // 3. 强制 s 严格递增
            double current_point_s = start_s + (i * step_size);

            // 边界检查：如果超过参考线末端则停止
            if (current_point_s > reference_line.refer_line.back().rs)
            {
                break;
            }

            LocalPathPoint point_tmp;
            point_tmp.s = current_point_s;

            // 4. 设置默认速度参数（防止后续计算出现除以 0）
            point_tmp.ds_dt = 1.0; // 即使没写速度规划，也给个默认值
            point_tmp.dds_dt = 0.0;
            point_tmp.l = 0.0;
            point_tmp.dl_ds = 0.0;
            point_tmp.ddl_ds = 0.0;

            // 5. 匹配决策中心给出的 SL 路径（SL 投影逻辑）
            const int sl_points_size = decision->sl_points().size();
            for (int j = 0; j < sl_points_size - 1; j++)
            {
                const double seg_start_s = decision->sl_points()[j].s_;
                const double seg_end_s = decision->sl_points()[j + 1].s_;

                if (current_point_s >= seg_start_s && current_point_s < seg_end_s)
                {
                    // 计算五次多项式系数并求出 l, dl_ds, ddl_ds
                    // 这里建议将 start_s, end_s 等参数传入 quintic_polynomial
                    const Eigen::Vector<double, 6> a = PolynomialCurve::quintic_polynomial(
                        seg_start_s, decision->sl_points()[j].l_, 0.0, 0.0,
                        seg_end_s, decision->sl_points()[j + 1].l_, 0.0, 0.0);

                    // 注意：多项式变量应为 (current_point_s - seg_start_s) 以保证数值稳定性
                    // 如果你的函数是基于绝对 s，请确保公式一致
                    double ds_rel = current_point_s;
                    double ds_rel_2 = ds_rel * ds_rel;
                    double ds_rel_3 = ds_rel_2 * ds_rel;
                    double ds_rel_4 = ds_rel_3 * ds_rel;
                    double ds_rel_5 = ds_rel_4 * ds_rel;

                    point_tmp.l = a(0) + a(1) * ds_rel + a(2) * ds_rel_2 + a(3) * ds_rel_3 + a(4) * ds_rel_4 + a(5) * ds_rel_5;
                    // ... 其他导数计算 ...
                    break;
                }
            }
            local_path_.local_path.emplace_back(point_tmp);
        }

        // 平滑 todo: johan
        local_path_smoother_->smooth_local_path(local_path_);

        // 转笛卡尔坐标系
        tf2::Quaternion qtn;
        for (auto &point : local_path_.local_path)
        {
            // 计算路径点在参考线上的投影
            const double rs = point.s;
            const int match_index = Curve::find_match_point(reference_line, rs);
            const double rx = reference_line.refer_line[match_index].pose.pose.position.x;
            const double ry = reference_line.refer_line[match_index].pose.pose.position.y;
            const double rtheta = reference_line.refer_line[match_index].rtheta;
            const double rkappa = reference_line.refer_line[match_index].rkappa;
            const double rdkappa = reference_line.refer_line[match_index].rdkappa;

            ToCartensianInPutTP ft_to_ct_input;
            ft_to_ct_input.s = point.s;
            ft_to_ct_input.ds_dt = point.ds_dt;
            ft_to_ct_input.dds_dt = point.dds_dt;
            ft_to_ct_input.l = point.l;
            ft_to_ct_input.dl_ds = point.dl_ds;
            ft_to_ct_input.ddl_ds = point.ddl_ds;

            ToCartensianOutTP ft_to_ct_output;
            ft_to_ct_output.rs = rs;
            ft_to_ct_output.rx = rx;
            ft_to_ct_output.ry = ry;
            ft_to_ct_output.rtheta = rtheta;
            ft_to_ct_output.rkappa = rkappa;
            ft_to_ct_output.rdkappa = rdkappa;
            // 计算路径点在笛卡尔下的参数
            Curve::frenet_to_cartensian(ft_to_ct_input, ft_to_ct_output);

            point.pose.header = local_path_.header;
            point.pose.pose.position.x = ft_to_ct_output.x;
            point.pose.pose.position.y = ft_to_ct_output.y;
            point.theta = ft_to_ct_output.theta;
            point.kappa = ft_to_ct_output.kappa;

            qtn.setRPY(0.0, 0.0, point.theta);
            point.pose.pose.orientation.x = qtn.getX();
            point.pose.pose.orientation.y = qtn.getY();
            point.pose.pose.orientation.z = qtn.getZ();
            point.pose.pose.orientation.w = qtn.getW();
        }

        // 计算投影点坐标
        Curve::cal_projection_param(local_path_);

        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local path created, size: %ld", local_path_.local_path.size());
        return local_path_;
    }
    Path LocalPathPlanner::path_to_rviz()
    {
        local_path_rviz_.header = local_path_.header;
        local_path_rviz_.poses.clear();
        PoseStamped point_tmp;
        point_tmp.header = local_path_rviz_.header;
        for (const auto &point : local_path_.local_path)
        {
            point_tmp.pose = point.pose.pose;
            local_path_rviz_.poses.push_back(point_tmp);
        }

        return local_path_rviz_;
    }
    void LocalPathPlanner::init_local_path()
    {
        local_path_.header.frame_id = local_path_config_->pnc_map().frame_;
        local_path_.header.stamp = rclcpp::Clock().now();
        local_path_.local_path.clear();
    }
} // namespace Planning
