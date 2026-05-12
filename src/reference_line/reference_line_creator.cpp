#include "reference_line_creator.h"

namespace Planning
{
    ReferencelineCreator::ReferencelineCreator() // 创建参考线
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_creator created");
        // 读取配置文件
        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();
        // 创建平滑器
        reference_line_smoother_ = std::make_shared<ReferenceLineSmoother>();
    }

    Path ReferencelineCreator::reference_to_rviz()
    {
        
        refer_line_rviz_.header = refer_line_.header;
        refer_line_rviz_.poses.clear();
        PoseStamped point_tmp;
        for(const auto &point : refer_line_.refer_line){
            point_tmp.header = refer_line_.header;
            point_tmp.pose = point.pose.pose;
            refer_line_rviz_.poses.emplace_back(point_tmp);
        }
        return refer_line_rviz_;
    }

    void ReferencelineCreator::init_reference_line()
    {
        refer_line_.header.frame_id = reference_line_config_->pnc_map().frame_;
        refer_line_.header.stamp = rclcpp::Clock().now();
        refer_line_.refer_line.clear();
    }

    Referline ReferencelineCreator::create_reference_line(const Path &global_path, const PoseStamped &target_point)
    {
        if (global_path.poses.empty())
        {
            return refer_line_;
        }

        // 找匹配点
        match_point_index_ = Curve::find_match_point(global_path, last_match_point_index_, target_point);
        last_match_point_index_ = match_point_index_; // 更新上一帧
        if (match_point_index_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "find match point index failed");
        }
        // 计算最前点和最后点全局路径下下标
        const int global_path_size = global_path.poses.size();
        front_index_ = (global_path_size - 1 >= match_point_index_ + reference_line_config_->refer_line().front_size_)
                           ? (match_point_index_ + reference_line_config_->refer_line().front_size_)
                           : (global_path_size - 1);
        back_index_ = (match_point_index_ - reference_line_config_->refer_line().back_size_ >= 0)
                          ? (match_point_index_ - reference_line_config_->refer_line().back_size_)
                          : 0;        

        // 填充参考线
        init_reference_line();
        ReferlinePoint point_tmp;
        for (int i = back_index_; i <= front_index_; i++)
        {
            point_tmp.pose = global_path.poses[i];
            refer_line_.refer_line.emplace_back(point_tmp);
        }
        //  平滑参考线
        reference_line_smoother_->smooth_reference_line(refer_line_);

        // 计算投影点参数
        Curve::cal_projection_param(refer_line_);
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference line create, match_point_index = %d, front_index = %d, back_index = %d, size = %ld",
                    match_point_index_, front_index_, back_index_, refer_line_.refer_line.size());
        return refer_line_;
    }

} // namespace Planning