#include <memory>

#include "../include/ars408_radar/ars408_radar_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto ars408_radar_node = std::make_shared<FHAC::ars408_radar>(options);
    exec.add_node(ars408_radar_node->get_node_base_interface());

    RCLCPP_INFO(ars408_radar_node->get_logger(), "This is my ARS408 RADAR");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}