#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pointcloud_compressor/msg/pattern_dictionary.hpp>

#include "decompressed/pattern_dictionary_decompressor.hpp"
#include "decompressed/marker_visualizer.hpp"

namespace decompressed {

/**
 * @brief ROS2 node for decompressing and visualizing pattern dictionary data
 */
class DecompressedViewerNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    DecompressedViewerNode();
    
    ~DecompressedViewerNode() = default;
    
private:
    /**
     * @brief Declare all ROS parameters
     */
    void declareParameters();
    
    /**
     * @brief Get all parameter values
     */
    void getParameters();
    
    /**
     * @brief Setup ROS subscribers and publishers
     */
    void setupROSCommunication();
    
    /**
     * @brief Callback for PatternDictionary messages
     * @param msg PatternDictionary message
     */
    void patternDictionaryCallback(const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg);
    
    /**
     * @brief Process PatternDictionary and create markers
     * @param msg PatternDictionary message
     * @return MarkerArray with pattern markers
     */
    visualization_msgs::msg::MarkerArray processPatternDictionary(
        const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg);
    
    /**
     * @brief Get voxel size from message or use default
     * @param msg PatternDictionary message
     * @return Voxel size in meters
     */
    double getVoxelSize(const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg) const;
    
    /**
     * @brief Get patterns from cache or decompress
     * @param msg PatternDictionary message
     * @return Vector of decompressed pattern blocks
     */
    std::vector<Block3D<8>> getPatterns(const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg);
    
    /**
     * @brief Check if message has valid spatial information
     * @param msg PatternDictionary message
     * @return True if spatial info is valid
     */
    bool hasSpatialInfo(const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg) const;
    
    /**
     * @brief Create spatial pattern markers
     * @param msg PatternDictionary message
     * @param patterns Decompressed pattern blocks
     * @param voxel_size Voxel size in meters
     * @return MarkerArray with spatial markers
     */
    visualization_msgs::msg::MarkerArray createSpatialMarkers(
        const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg,
        const std::vector<Block3D<8>>& patterns,
        double voxel_size);
    
    /**
     * @brief Log information about published markers
     * @param marker_array MarkerArray to log
     */
    void logMarkerInfo(const visualization_msgs::msg::MarkerArray& marker_array);
    
    /**
     * @brief Calculate hash for pattern data
     * @param data Pattern dictionary data
     * @return Hash value
     */
    size_t calculatePatternHash(const std::vector<uint8_t>& data) const;
    
    // Parameters
    std::string pattern_dictionary_topic_;
    std::string pattern_markers_topic_;
    std::string frame_id_;
    bool show_pattern_visualization_;
    
    // Components
    std::unique_ptr<PatternDictionaryDecompressor> pattern_decompressor_;
    std::unique_ptr<MarkerVisualizer> pattern_visualizer_;
    
    // State management
    std::vector<Block3D<8>> cached_patterns_;
    size_t cached_pattern_hash_;
    std::chrono::steady_clock::time_point last_publish_time_;
    std::chrono::milliseconds min_publish_interval_;
    
    // ROS communication
    rclcpp::Subscription<pointcloud_compressor::msg::PatternDictionary>::SharedPtr pattern_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pattern_markers_pub_;
};

} // namespace decompressed