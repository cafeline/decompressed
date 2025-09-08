#include <rclcpp/rclcpp.hpp>
#include "decompressed/decompressed_viewer_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<decompressed::DecompressedViewerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}