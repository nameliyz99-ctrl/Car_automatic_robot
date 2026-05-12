#ifndef PNC_MAP_SERVER_H_
#define PNC_MAP_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "pnc_map_creator_straight.h"
#include "pnc_map_creator_sturn.h"

namespace Planning
{
    using base_msgs::srv::PNCMapService;
    using std::placeholders::_1;
    using std::placeholders::_2;
    class PNCMapServer : public rclcpp::Node
    {
    public:
        PNCMapServer();
    private:
        std::shared_ptr<PNCMapCreatorBase> map_creater_;//地图创建器基类指针，实际使用时可以指向不同的地图创建器实现（如直道、S弯等），方便扩展和维护
        rclcpp::Publisher<PNCMap>::SharedPtr map_pub_;//地图发布器
        rclcpp::Publisher<MarkerArray>::SharedPtr map_rviz_pub_;//地图markerarry发布其实可以和上一个合成一个发布器，但为了代码清晰，这里分开了   
        rclcpp::Service<PNCMapService>::SharedPtr map_service_;//地图服务器
    private:
        void response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request, 
                                       const std::shared_ptr<PNCMapService::Response> response);
    };
} // namespace Planning
#endif // PNC_MAP_SERVER_H_
