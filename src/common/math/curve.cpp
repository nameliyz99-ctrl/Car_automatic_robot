#include "curve.h"

namespace Planning
{
    double Curve::NormalizeAngle(const double &angle)
    {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0)
        {
            a += (2.0 * M_PI);
        }

        return a - M_PI;
    }

    void Curve::cartensian_to_frenet(const ToFrenetInPutTP &cartensian_input, ToFrenetOutTP &frenet_output)
    {
        // 计算s
        frenet_output.s = cartensian_input.rs;
        // 计算l
        const double dx = cartensian_input.x - cartensian_input.rx;
        const double dy = cartensian_input.y - cartensian_input.ry;

        const double cos_theta_r = std::cos(cartensian_input.rtheta);
        const double sin_theta_r = std::sin(cartensian_input.rtheta);

        const double cross_r_x = cos_theta_r * dy - sin_theta_r * dx;
        frenet_output.l = std::copysign(std::hypot(dx, dy), cross_r_x);
        // 计算l'=dl/ds
        const double delta_theta = cartensian_input.theta - cartensian_input.rtheta;
        const double tan_delta_theta = std::tan(delta_theta);
        const double cos_delta_theta = std::cos(delta_theta);
        const double sin_delta_theta = std::sin(delta_theta);
        const double one_minus_kappa_l = 1 - cartensian_input.rkappa * frenet_output.l;
        frenet_output.dl_ds = one_minus_kappa_l * tan_delta_theta;

        // 计算l''=ddl/ds
        const double kappa_l_prime = cartensian_input.rdkappa * frenet_output.l + cartensian_input.rkappa * frenet_output.dl_ds;
        const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * cartensian_input.kappa - cartensian_input.rkappa;
        frenet_output.ddl_ds = -kappa_l_prime * tan_delta_theta + one_minus_kappa_l / (cos_delta_theta * cos_delta_theta) * delta_theta_prime;
        // 计算ds/dt
        frenet_output.ds_dt = cartensian_input.speed * cos_delta_theta / one_minus_kappa_l;
        // 计算dds/dt
        frenet_output.dds_dt = (cartensian_input.a * cos_delta_theta - (frenet_output.ds_dt * frenet_output.ds_dt) * (frenet_output.dl_ds * delta_theta_prime - kappa_l_prime)) / one_minus_kappa_l;
        ;
        // 计算dl_dt
        frenet_output.dl_dt = cartensian_input.speed * sin_delta_theta;
        // 计算ddl_dt
        frenet_output.ddl_dt = cartensian_input.a * sin_delta_theta;
    }

    void Curve::frenet_to_cartensian(const ToCartensianInPutTP &frenet_input, ToCartensianOutTP &cartensian_output)
    {
        // 判断s和rs是否足够近
        if (std::fabs(frenet_input.s - cartensian_output.rs) > delta_s_min)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "reference point s and projection rs dont't match! rs = %.2f, s = %.2f", cartensian_output.rs, frenet_input.s);
            return;
        }
        // 计算x y
        const double cos_theta_r = std::cos(cartensian_output.rtheta);
        const double sin_theta_r = std::sin(cartensian_output.rtheta);
        cartensian_output.x = cartensian_output.rx - sin_theta_r * frenet_input.l;
        cartensian_output.y = cartensian_output.ry + cos_theta_r * frenet_input.l;
        
        // 计算theta
        const double one_minus_kappa_l = 1 - cartensian_output.rkappa * frenet_input.l;
        const double tan_delta_theta = frenet_input.dl_ds / one_minus_kappa_l;
        const double delta_theta = std::atan2(frenet_input.dl_ds, one_minus_kappa_l);
        const double cos_delta_theta = std::cos(delta_theta);
        cartensian_output.theta = NormalizeAngle(delta_theta + cartensian_output.rtheta);
        // 计算kappa
        const double kappa_l_prime = cartensian_output.rdkappa * frenet_input.l + cartensian_output.rkappa * frenet_input.dl_ds;
        cartensian_output.kappa = ((frenet_input.ddl_ds + kappa_l_prime * tan_delta_theta) * (cos_delta_theta * cos_delta_theta) / one_minus_kappa_l + cartensian_output.rkappa) *
                                  cos_delta_theta / one_minus_kappa_l;
        // 计算speed
        cartensian_output.speed = std::hypot(frenet_input.ds_dt * one_minus_kappa_l, frenet_input.ds_dt*frenet_input.dl_ds);
        // 计算a
        const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * cartensian_output.kappa - cartensian_output.rkappa;
        cartensian_output.a = frenet_input.dds_dt * one_minus_kappa_l / cos_delta_theta +
                              (frenet_input.ds_dt * frenet_input.ds_dt) / cos_delta_theta *
                                  (frenet_input.dl_ds * delta_theta_prime - kappa_l_prime);
    }

    int Curve::find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point)
    {
        const int path_size = path.poses.size();
        if (path_size <= 1)
        {
            return path_size - 1;
        }
        double min_dis = std::numeric_limits<double>::max();
        int closet_index = -1;
        const int threshould_jump{100};
        for (int i = 0; i < path_size; i++)
        {
            double dis = std::hypot(path.poses[i].pose.position.x - target_point.pose.position.x,
                                    path.poses[i].pose.position.y - target_point.pose.position.y);
            if (dis < min_dis)
            {
                if (abs(i - last_match_point_index) > threshould_jump)
                {
                    continue;
                }
                min_dis = dis;
                closet_index = i;
            }
        }

        return closet_index;
    }

    // 在参考线上寻找匹配点
    int Curve::find_match_point(const Referline &refer_line, const PoseStamped &target_point)
    {
        const int path_size = refer_line.refer_line.size();
        if (path_size <= 1)
        {
            return path_size - 1;
        }
        double min_dis = std::numeric_limits<double>::max();
        int closet_index = -1;
        const int threshould_jump{100};
        for (int i = 0; i < path_size; i++)
        {
            double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.x - target_point.pose.position.x,
                                    refer_line.refer_line[i].pose.pose.position.y - target_point.pose.position.y);
            if (dis < min_dis)
            {
                min_dis = dis;
                closet_index = i;
            }
        }
        return closet_index;
    }

    // 在路径上寻找匹配点
    int Curve::find_match_point(const LocalPath &path, const PoseStamped &target_point)
    {
        const int path_size = path.local_path.size();
        if (path_size <= 1)
        {
            return path_size - 1;
        }
        double min_dis = std::numeric_limits<double>::max();
        int closet_index = -1;
        for (int i = 0; i < path_size; i++)
        {
            double dis = std::hypot(path.local_path[i].pose.pose.position.x - target_point.pose.position.x,
                                    path.local_path[i].pose.pose.position.y - target_point.pose.position.y);
            if (dis < min_dis)
            {
                min_dis = dis;
                closet_index = i;
            }
        }
        return closet_index;
    }
    int Curve::find_match_point(const Referline &path, const double &rs)
    {
        const int path_size = path.refer_line.size();
        if (path_size <= 1)
        {
            return path_size - 1;
        }
        double min_delta_s = std::numeric_limits<double>::max();
        int closet_index = -1;
        const int threshould_jump{100};
        for (int i = 0; i < path_size; i++)
        {
            double delta_s = std::fabs(rs - path.refer_line[i].rs);
            if (delta_s < min_delta_s)
            {
                min_delta_s = delta_s;
                closet_index = i;
            }
        }
        return closet_index;
    }
    void Curve::find_projection_point(const Referline &referline, const PoseStamped &target_point, ToFrenetInPutTP &projection_point)
    {
        // 用匹配点近似替代投影点，前提：参考线足够密且平滑
        const int match_index = find_match_point(referline, target_point);
        if (match_index < 0)
        {
            return;
        }
        // x y没问题说明这里匹配点算的没问题 s有问题，那就是参考线部分有问题
        projection_point.rx = referline.refer_line[match_index].pose.pose.position.x;
        projection_point.ry = referline.refer_line[match_index].pose.pose.position.y;
        
        projection_point.rs = referline.refer_line[match_index].rs;
        // RCLCPP_INFO(rclcpp::get_logger("curve"),"match_index: %d, rs: %.2f", match_index,projection_point.rs);
        projection_point.rtheta = referline.refer_line[match_index].rtheta;
        projection_point.rkappa = referline.refer_line[match_index].rkappa;
        projection_point.rdkappa = referline.refer_line[match_index].rdkappa;
    }

    void Curve::find_projection_point(const LocalPath &path, const PoseStamped &target_point, ToFrenetInPutTP &projection_point)
    {
        const int match_index = find_match_point(path, target_point);
        if (match_index < 0)
        {
            return;
        }
        projection_point.rx = path.local_path[match_index].pose.pose.position.x;
        projection_point.ry = path.local_path[match_index].pose.pose.position.y;
        projection_point.rs = path.local_path[match_index].rs;
        projection_point.rtheta = path.local_path[match_index].rtheta;
        projection_point.rkappa = path.local_path[match_index].rkappa;
        projection_point.rdkappa = path.local_path[match_index].rdkappa;
    }

    // 计算参考线参数（参考线）
    void Curve::cal_projection_param(Referline &refer_line)
    {
        const int path_size = refer_line.refer_line.size();
        if (path_size < 3)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "refer_line too short");
            return;
        }
        // 计算rs
        double rs{0.0};
        for (int i = 0; i < path_size; i++)
        {
            if (i == 0)
            {
                rs = 0.0;
            }
            else
            {
                // 用两点距离近似累积弧线
                rs += std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                 refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
            }
            refer_line.refer_line[i].rs = rs;

        }
        // 计算航向角 theta
        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1)
            {
                refer_line.refer_line[i].rtheta =
                    std::atan2(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                               refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
            }
            else
            {
                // 最后一个点用前一个点计算
                refer_line.refer_line[i].rtheta =
                    std::atan2(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                               refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
            }
            // refer_line.refer_line[i].rs = rs;
        }
        // 计算曲率   theta[i+1]-theta[i] / ds
        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1)
            {
                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                              refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                if (dis < kMathEpsilon)
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                    continue;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i + 1].rtheta - refer_line.refer_line[i].rtheta) / dis;
                }
            }
            else
            {
                // 最后一个点用前一个点计算
                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                if (dis < kMathEpsilon)
                {
                    refer_line.refer_line[i].rkappa = 0.0;
                    continue;
                }
                else
                {
                    refer_line.refer_line[i].rkappa = (refer_line.refer_line[i].rtheta - refer_line.refer_line[i - 1].rtheta) / dis;
                }
            }
        }

        // 计算曲率变化率

        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1)
            {
                const double dis = std::hypot(refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
                                              refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                if (dis < kMathEpsilon)
                {
                    refer_line.refer_line[i].rdkappa = 0.0;
                    continue;
                }
                else
                {
                    refer_line.refer_line[i].rdkappa = (refer_line.refer_line[i + 1].rkappa - refer_line.refer_line[i].rkappa) / dis;
                }
            }
            else
            {
                // 最后一个点用前一个点计算
                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                if (dis < kMathEpsilon)
                {
                    refer_line.refer_line[i].rdkappa = 0.0;
                    continue;
                }
                else
                {
                    refer_line.refer_line[i].rdkappa = (refer_line.refer_line[i].rkappa - refer_line.refer_line[i - 1].rkappa) / dis;
                }
            }
        }
    }

    // 计算投影点参数（路径）
    void Curve::cal_projection_param(LocalPath &local_path)
    {
        const int path_size = local_path.local_path.size();
        if (path_size < 3)
        {
            RCLCPP_ERROR(rclcpp::get_logger("math"), "local_path too short");
            return;
        }
        // 计算rs
        double rs{0.0};
        for (int i = 0; i < path_size; i++)
        {
            if (i == 0)
            {
                rs = 0.0;
            }
            else
            {
                // 用两点距离近似累积弧线
                rs += std::hypot(local_path.local_path[i].pose.pose.position.y - local_path.local_path[i - 1].pose.pose.position.y,
                                 local_path.local_path[i].pose.pose.position.x - local_path.local_path[i - 1].pose.pose.position.x);
            }
            local_path.local_path[i].rs = rs;
        }
        // 计算航向角和曲率
        for (int i = 0; i < path_size; i++)
        {
            local_path.local_path[i].rtheta = local_path.local_path[i].theta;
            local_path.local_path[i].rkappa = local_path.local_path[i].kappa;
        }
        // 计算曲率变化率
        for (int i = 0; i < path_size; i++)
        {
            if (i < path_size - 1)
            {
                const double dis = std::hypot(local_path.local_path[i + 1].pose.pose.position.y - local_path.local_path[i].pose.pose.position.y,
                                              local_path.local_path[i + 1].pose.pose.position.x - local_path.local_path[i].pose.pose.position.x);
                if (dis < kMathEpsilon)
                {
                    local_path.local_path[i].rdkappa = 0.0;
                    continue;
                }
                else
                {
                    local_path.local_path[i].rdkappa = (local_path.local_path[i + 1].rkappa - local_path.local_path[i].rkappa) / dis;
                }
            }
            else
            {
                // 最后一个点用前一个点计算
                const double dis = std::hypot(local_path.local_path[i].pose.pose.position.y - local_path.local_path[i - 1].pose.pose.position.y,
                                              local_path.local_path[i].pose.pose.position.x - local_path.local_path[i - 1].pose.pose.position.x);
                if (dis < kMathEpsilon)
                {
                    local_path.local_path[i].rdkappa = 0.0;
                    continue;
                }
                else
                {
                    local_path.local_path[i].rdkappa = (local_path.local_path[i].rkappa - local_path.local_path[i - 1].rkappa) / dis;
                }
            }
        }
    }
}
