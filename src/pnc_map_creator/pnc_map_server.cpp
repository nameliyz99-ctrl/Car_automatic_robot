#include "pnc_map_server.h"

namespace Planning
{
    PNCMapServer::PNCMapServer() : Node("pnc_map_server_node") // 全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "pnc_map_server_node created");
        // 创建地图发布器
        map_pub_ = this->create_publisher<PNCMap>("pnc_map", 10);
        map_rviz_pub_ = this->create_publisher<MarkerArray>("pnc_map_markerarray", 10);
        // 创建地图服务
        map_service_ = this->create_service<PNCMapService>(
            "pnc_map_server",
            std::bind(&PNCMapServer::response_pnc_map_callback, this, _1, _2)
        );
    }
    // 相应并发布地图
    void PNCMapServer::response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request,
                                                 const std::shared_ptr<PNCMapService::Response> response)
    {
        switch (request->map_type)
        {
        case static_cast<int>(PNCMapType::STRAIGHT): // 直道
            map_creater_ = std::make_shared<PNCMapCreatorStraight>();
            break;
        case static_cast<int>(PNCMapType::STURN): // S弯
            map_creater_ = std::make_shared<PNCMapCreatorSTurn>();
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown map type requested: %d", request->map_type);
            return;
        }
        const auto pnc_map = map_creater_->creat_pnc_map(); // 生成地图
        response->pnc_map = pnc_map;                        // 填充响应
        map_pub_->publish(pnc_map);                         // 发布地图
        RCLCPP_INFO(this->get_logger(), "pnc_map published");
        // 发布RViz可视化MarkerArray
        // 这里可以根据pnc_map的内容创建对应的Marker并添加到marker_array中，以下是一个示例
        // 生成地图
        const auto pnc_map_makerarray = map_creater_->pnc_map_markerarray();
        map_rviz_pub_->publish(pnc_map_makerarray);
        RCLCPP_INFO(this->get_logger(), "PNCMap RVIZ publicshed");
    }
} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}