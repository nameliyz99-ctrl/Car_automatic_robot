#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

namespace Planning
{
#define ToCartensianInPutTP ToFrenetOutTP
#define ToCartensianOutTP ToFrenetInPutTP
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;
    constexpr double delta_s_min = 1.0;
    constexpr double kMathEpsilon = 1e-6;
    struct ToFrenetInPutTP
    {
        // 目标点在笛卡尔坐标系下的参数
        double x;
        double y;
        double theta;
        double speed;
        double a;
        double kappa;
        // 目标点在参考线的投影点在笛卡尔坐标系下的参数
        double rs;
        double rx;
        double ry;
        double rtheta;
        double rkappa;
        double rdkappa;
    };
    struct ToFrenetOutTP
    {
        double s;
        double ds_dt;
        double dds_dt;
        double l;
        double dl_ds;
        double dl_dt;
        double ddl_ds;
        double ddl_dt;
    };
    class Curve // 曲线
    {
    public:
        Curve() = default;

        static double NormalizeAngle(const double &angle); //[-pi,pi]

        // 笛卡尔转frenet
        static void cartensian_to_frenet(const ToFrenetInPutTP &cartensian_input, ToFrenetOutTP &frenet_output);
        // frenet转笛卡尔
        static void frenet_to_cartensian(const ToCartensianInPutTP &frenet_input, ToCartensianOutTP &cartensian_output);

        // 找匹配点下标
        static int find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point); // 利用上一帧
        static int find_match_point(const Referline &refer_line, const PoseStamped &target_point);                         // 在参考线上查找匹配点下标
        static int find_match_point(const LocalPath &path, const PoseStamped &target_point);                               // 在路径上查找匹配点下标
        static int find_match_point(const Referline &path, const double &rs);                                              // 通过rs查找匹配点下标

        // 寻找投影点
        static void find_projection_point(const Referline &referline, const PoseStamped &target_point, ToFrenetInPutTP &projection_point);
        static void find_projection_point(const LocalPath &path, const PoseStamped &target_point, ToFrenetInPutTP &projection_point);
        // 计算投影点参数
        static void cal_projection_param(Referline &refer_line); // 参考线
        static void cal_projection_param(LocalPath &local_path); // 路径
    };
} // namespace Planning
#endif // CURVE_H_