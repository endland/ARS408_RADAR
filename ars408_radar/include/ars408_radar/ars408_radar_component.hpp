#ifndef COMPOSITON__RADAR_ARS408_COMPONENT_HPP_
#define COMPOSTION__RADAR_ARS408_COMPONENT_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <map>

//#include "visibility_control.h"

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <ars408_radar_msgs/msg/object_list.hpp>
#include <ars408_radar_msgs/msg/cluster_list.hpp>
#include <ars408_radar_msgs/msg/radar_state.hpp>
#include <ars408_radar_msgs/msg/cluster_status.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_factory.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/timer.hpp>
#include <rmw/qos_profiles.h>


#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include "ros2socketcan_bridge/ros2socketcan.h"
#include <ars408_radar/ars_408_can_defines.h>

constexpr char DEFAULT_NODE_NAME[] = "ARS408_RADAR";

typedef unsigned char ubyte;
typedef unsigned short int uword;

//using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
//using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

namespace FHAC
{


    class ars408_radar : public rclcpp_lifecycle::LifecycleNode
    {
        public:
        //ARS408_RADAR_PUBLIC
        ars408_radar(const rclcpp::NodeOptions& options);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& );
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& );
            



        private:

        //### Task2
        // 1. Create CAN Channel object
        ros2socketcan canChannel0;

        // 2. Create Publisher
        rclcpp::QoS qos{10};
        std::string pub_marker_array_topic_name = "/radar/marker_array";
        std::string pub_object_list_topic_name = "/radar/objectlist";
        std::string pub_tf_topic_name = "/tf";
        std::string frame_id_ = "/radar_link";

        rclcpp_lifecycle::LifecyclePublisher<ars408_radar_msgs::msg::ObjectList>::SharedPtr object_list_publisher_;
        rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

        //std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

        //3. create can receive callback
        void can_receive_callback(const can_msgs::msg::Frame);

        //4. create handle object list
        void handle_object_list(const can_msgs::msg::Frame);

        //5. create publish object map
        void publish_object_map();

        //6. create map container for object list
        std::map<int, ars408_radar_msgs::msg::Object> object_map_;

        //7. create data structure for radar object list
        ars408_radar_msgs::msg::ObjectList object_list_;

        // additional variables
        int operation_mode_;
    };
}

#endif
