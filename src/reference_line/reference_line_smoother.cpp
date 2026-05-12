#include "reference_line_smoother.h"

namespace Planning
{
    ReferenceLineSmoother::ReferenceLineSmoother()
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_smoother created");
        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();
    }
    void ReferenceLineSmoother::smooth_reference_line(Referline &refer_line)
    {
        const int n = refer_line.refer_line.size();
        if (n < 3)
        {
            return;
        }

        // P矩阵
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d W1 = 2.0 * w1 * I;
        Eigen::Matrix2d W2 = 2.0 * w2 * I;
        Eigen::Matrix2d W3 = 2.0 * w3 * I;

        Eigen::Matrix2d block1 = W1 + W2 + W3;
        Eigen::Matrix2d block2 = -2.0 * W1 - W2;
        Eigen::Matrix2d block3 = -4.0 * W1 - W2;
        Eigen::Matrix2d block4 = 5.0 * W1 + 2.0 * W2 + W3;
        Eigen::Matrix2d block5 = 6.0 * W1 + 2.0 * W2 + W3;

        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(2 * n, 2 * n);
        if (n == 3)
        {
            /*
            W1+W2+W3  -2W1-W2      W1
            -2W1-W2   4W1+2W2+W3  -2W1-W2
             W1       -2W1-W2     W1+W2+W3
            */
            P_tmp.block<2, 2>(0, 0) = block1;
            P_tmp.block<2, 2>(0, 2) = block2;
            P_tmp.block<2, 2>(0, 4) = W1;
            P_tmp.block<2, 2>(2, 0) = block2;
            P_tmp.block<2, 2>(2, 2) = 4.0 * W1 + 2.0 * W2 + W3;
            P_tmp.block<2, 2>(2, 4) = block2;
            P_tmp.block<2, 2>(4, 4) = block1;
        }
        else
        {
            /*
            W1+W2+W3  -2W1-W2      W1          0        ...             0
                      5W1+2W2+W3  -4W1-W2      W1       ...             0
                                  6W1+2W2+W3 -4W1-W2    W1              0
                                               ...      ...             0
                                           6W1+2W2+W3  -4W1-W2           W1
                                                       5W1+2W2+W3   -2W1-W2
                                                                    W1+W2+W3
            */
            for (int i = 0; i < n; i++)
            {
                if (i == 0)
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block1;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
                else if (i == 1)
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block4;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
                else if (i == n - 2)
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block4;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
                }
                else if (i == n - 1)
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block1;
                }
                else
                {
                    P_tmp.block<2, 2>(i * 2, i * 2) = block5;
                    P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
                    P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
                }
            }
        }

        P_tmp = P_tmp.selfadjointView<Eigen::Upper>();      // 通过上三角构造对称矩阵
        Eigen::SparseMatrix<double> P = P_tmp.sparseView(); // 稀疏矩阵

        // A矩阵
        Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(2 * n, 2 * n);
        Eigen::SparseMatrix<double> A = A_tmp.sparseView();

        // 原始点
        Eigen::VectorXd X(2 * n);
        for (int i = 0; i < n; i++)
        {
            X(2 * i) = refer_line.refer_line[i].pose.pose.position.x;
            X(2 * i + 1) = refer_line.refer_line[i].pose.pose.position.y;
        }
        Eigen::VectorXd Q = -2.0 * X;                                 // 一次项矩阵
        Eigen::VectorXd buff = Eigen::VectorXd::Constant(2 * n, 0.2); // 偏差矩阵
        buff(0) = buff(1) = buff(2 * n - 2) = buff(2 * n - 1) = 0.0;  // 首位点不容许误差
        Eigen::VectorXd lowerBound = X - buff;                        // 不等式约束下边界
        Eigen::VectorXd upperBound = X + buff;                        // 不等式约束上边界

        // OsqpEigen::Solver solver;
        // solver.settings()->setVerbosity(false);
        // solver.settings()->setWarmStart(true);

        // solver.data()->setNumberOfVariables(2 * n);   // 变量数
        // solver.data()->setNumberOfConstraints(2 * n); // 约束数
        // if (!solver.data()->setHessianMatrix(P))
        // {
        //     return;
        // }
        // if (!solver.data()->setGradient(Q))
        // {
        //     return;
        // }
        // if (!solver.data()->setLinearConstraintsMatrix(A))
        // {
        //     return;
        // }
        // if (!solver.data()->setLowerBound(lowerBound))
        // {
        //     return;
        // }
        // if (!solver.data()->setUpperBound(upperBound))
        // {
        //     return;
        // }

        // if (!solver.initSolver())
        // {
        //     return;
        // }

        Eigen::VectorXd QPSolution;
        if(!OsqpSolver(n, P, Q, A, lowerBound, upperBound, QPSolution)){
            return;
        }

        // if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        // {
        //     return;
        // }
        // QPSolution = solver.getSolution();
        // RCLCPP_INFO(rclcpp::get_logger("planning"), "QPSolution: %f, %f", QPSolution(0), QPSolution(1));
        for (int i = 0; i < n; i++)
        {
            refer_line.refer_line[i].pose.pose.position.x = QPSolution(2 * i);
            refer_line.refer_line[i].pose.pose.position.y = QPSolution(2 * i + 1);
        }
    }

    bool ReferenceLineSmoother::OsqpSolver(const int &n,const Eigen::SparseMatrix<double> &P, Eigen::VectorXd Q, const Eigen::SparseMatrix<double> &A, Eigen::VectorXd lowerBound, Eigen::VectorXd upperBound, Eigen::VectorXd &QPsolution)
    {
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);

        solver.data()->setNumberOfVariables(2 * n);   // 变量数
        solver.data()->setNumberOfConstraints(2 * n); // 约束数
        if (!solver.data()->setHessianMatrix(P))
        {
            return false;
        }
        // Eigen::Map<const Eigen::VectorXd> Q_map(Q.data(), Q.size());
        if (!solver.data()->setGradient(Q))
        {
            return false;
        }
        if (!solver.data()->setLinearConstraintsMatrix(A))
        {
            return false;
        }
        // Eigen::Map<const Eigen::VectorXd> lowerBound_map(lowerBound.data(), lowerBound.size());
        if (!solver.data()->setLowerBound(lowerBound))
        {
            return false;
        }
        Eigen::Map<const Eigen::VectorXd> upperBound_map(upperBound.data(), upperBound.size());
        if (!solver.data()->setUpperBound(upperBound))
        {
            return false;
        }

        if (!solver.initSolver())
        {
            return false;
        }

        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        {
            return false;
        }
        QPsolution = solver.getSolution();
        // RCLCPP_INFO(rclcpp::get_logger("planning"), "smooth reference line success");
        return true;
    }

} // namespace Planning