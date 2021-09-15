/** Authors: 
    Narendiran CG <narendiran.cg@newspace.co.in> 
    Raghunandan V <raghunandan.v@newspace.co.in>

    Description:
    Node to create custom static bridges for known topics used in interplane communication 
*/

#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"

// #define BOOST_BIND_NO_PLACEHOLDERS

// include ROS 1
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
#pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"

// Global variable
std::set<std::string> ip_set;
std::map<std::string, ros1_bridge::BridgeHandles> bridge_handle_dict;

// Callback function to parse the incoming IP list and store it in the global var ip_set
void ip_list_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string delimiter = ",";
    std::string token;
    size_t pos = 0;
    // RCLCPP_INFO(rclcpp::get_logger(), "I heard: '%s'", msg->data.c_str());
    std::string ip_list_str = msg->data.c_str();

    while ((pos = ip_list_str.find(delimiter)) != std::string::npos)
    {
        token = ip_list_str.substr(0, pos);

        if (ip_set.find(token) == ip_set.end())
        {
            ip_set.insert(token);
        }
        ip_list_str.erase(0, pos + delimiter.length());
    }
    if (ip_set.find(ip_list_str) == ip_set.end())
    {
        ip_set.insert(ip_list_str);
        std::cout << ip_list_str << std::endl;
    }
}



int main(int argc, char *argv[])
{
    // ROS 1 node
    ros::init(argc, argv, "custom_static_bridge");
    ros::NodeHandle ros1_node;

    // ROS 2 node
    rclcpp::init(argc, argv);
    auto ros2_node = rclcpp::Node::make_shared("custom_static_bridge");

    std::string ros1_type_name = "std_msgs/String";
    std::string ros2_type_name = "std_msgs/msg/String";
    size_t queue_size = 10;

    // IP list subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ = 
        ros2_node->create_subscription<std_msgs::msg::String>("/ip_list", 10, &ip_list_callback);

    // Timer callback to check for new IPs periodically and add bridges:
    auto timer_callback = [
    &ros1_node, ros2_node,
    ros1_type_name, ros2_type_name,
    queue_size]() -> void
    {
        for (std::string ip : ip_set)
        {
            if (bridge_handle_dict.find(ip) == bridge_handle_dict.end())
            {
                std::string topic = ip + "/interplane_data";
                bridge_handle_dict.insert({ip, 
                    ros1_bridge::create_bidirectional_bridge(
                            ros1_node, 
                            ros2_node, 
                            ros1_type_name, 
                            ros2_type_name, 
                            topic, 
                            queue_size)});
            }
        }
    };

    rclcpp::TimerBase::SharedPtr timer_ = ros2_node->create_wall_timer(std::chrono::milliseconds(500), timer_callback);

    // Bridge for the gcs data:
    bridge_handle_dict.insert({"fw_gcs", ros1_bridge::create_bidirectional_bridge(
                            ros1_node, 
                            ros2_node, 
                            ros1_type_name, 
                            ros2_type_name, 
                            "/fw_gcs_data", 
                            queue_size)});
    
    // Bridge for the ip list:
    bridge_handle_dict.insert({"ip_list", ros1_bridge::create_bidirectional_bridge(
                            ros1_node, 
                            ros2_node, 
                            ros1_type_name, 
                            ros2_type_name, 
                            "/ip_list", 
                            queue_size)});
    
    // ROS 1 asynchronous spinner
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    // ROS 2 spinning loop
    rclcpp::executors::SingleThreadedExecutor executor;
    while (ros1_node.ok() && rclcpp::ok())
    {
        executor.spin_node_once(ros2_node , std::chrono::milliseconds(1000));
    }

    return 0;
}
