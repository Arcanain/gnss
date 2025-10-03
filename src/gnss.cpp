#include "gnss/gnss_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnss::GnssNode>();//create node instance
    
    // create Executer
    rclcpp::executors::SingleThreadedExecutor executor;

    // add node to executor
    executor.add_node(node->get_node_base_interface());

    // spin the node
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}