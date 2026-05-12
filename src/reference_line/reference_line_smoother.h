#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "base_msgs/msg/referline.hpp"
#include <Eigen/Dense>           // eigen
#include <OsqpEigen/OsqpEigen.h> // osqp-eigen
#include <cmath>

namespace Planning
{
    using base_msgs::msg::Referline;
    class ReferenceLineSmoother // 参考线平滑
    {
    public:
        ReferenceLineSmoother();
        void smooth_reference_line(Referline &refer_line); // 平滑参考线，输入为原始参考线对象，输出为平滑参考线对象
        bool OsqpSolver(const int &n,
                        const Eigen::SparseMatrix<double> &P,
                        Eigen::VectorXd Q,
                        const Eigen::SparseMatrix<double> &A,
                        Eigen::VectorXd lowerBound,
                        Eigen::VectorXd upperBound,
                        Eigen::VectorXd &QPsolution);
    private:

    std::unique_ptr<ConfigReader> reference_line_config_; // 配置
    const double w1=100.0; // 参考线平滑权重
    const double w2=10.0; // 参考线平滑权重
    const double w3=1.0; // 参考线平滑权重
    };
} // namespace Planning
#endif // REFERENCE_LINE_SMOOTHER_H_