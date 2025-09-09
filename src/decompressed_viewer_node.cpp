#include "decompressed/decompressed_viewer_node.hpp"
#include <rclcpp/qos.hpp>
#include <map>
#include <algorithm>

namespace decompressed {

DecompressedViewerNode::DecompressedViewerNode()
    : Node("decompressed_viewer_node"),
      cached_pattern_hash_(0),
      min_publish_interval_(100)  // 100ms minimum between publishes
{
    // Declare parameters
    declareParameters();
    
    // Get parameters
    getParameters();
    
    // Initialize components
    pattern_decompressor_ = std::make_unique<PatternDictionaryDecompressor>();
    pattern_visualizer_ = std::make_unique<MarkerVisualizer>(frame_id_);
    
    // Initialize state
    last_publish_time_ = std::chrono::steady_clock::now();
    
    // Setup ROS communication
    setupROSCommunication();
    
    RCLCPP_INFO(this->get_logger(), "Decompressed Viewer Node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", pattern_dictionary_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", pattern_markers_topic_.c_str());
}

void DecompressedViewerNode::declareParameters() {
    this->declare_parameter("pattern_dictionary_topic", "pattern_dictionary");
    this->declare_parameter("pattern_markers_topic", "pattern_markers");
    this->declare_parameter("frame_id", "map");
    this->declare_parameter("show_pattern_visualization", true);
}

void DecompressedViewerNode::getParameters() {
    pattern_dictionary_topic_ = this->get_parameter("pattern_dictionary_topic").as_string();
    pattern_markers_topic_ = this->get_parameter("pattern_markers_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    show_pattern_visualization_ = this->get_parameter("show_pattern_visualization").as_bool();
}

void DecompressedViewerNode::setupROSCommunication() {
    // QoS settings
    auto qos = rclcpp::QoS(10)
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .history(rclcpp::HistoryPolicy::KeepLast);
    
    // Create subscriber
    pattern_sub_ = this->create_subscription<pointcloud_compressor::msg::PatternDictionary>(
        pattern_dictionary_topic_,
        qos,
        std::bind(&DecompressedViewerNode::patternDictionaryCallback, this, std::placeholders::_1)
    );
    
    // Create publisher
    pattern_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        pattern_markers_topic_,
        qos
    );
}

void DecompressedViewerNode::patternDictionaryCallback(
    const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg) 
{
    if (!show_pattern_visualization_) {
        return;
    }
    
    try {
        // Rate limiting
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_last_publish = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_publish_time_);
        
        if (time_since_last_publish < min_publish_interval_) {
            return;
        }
        
        // Process the pattern dictionary
        auto marker_array = processPatternDictionary(msg);
        
        // Publish if we have markers
        if (!marker_array.markers.empty()) {
            pattern_markers_pub_->publish(marker_array);
            last_publish_time_ = current_time;
            
            // Log marker information
            logMarkerInfo(marker_array);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error in pattern_dictionary_callback: %s", e.what());
    }
}

visualization_msgs::msg::MarkerArray DecompressedViewerNode::processPatternDictionary(
    const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg) 
{
    // Determine voxel size
    double voxel_size = getVoxelSize(msg);
    
    RCLCPP_INFO(this->get_logger(), "Processing %u patterns with voxel_size=%.3f",
                msg->num_patterns, voxel_size);
    
    // Get patterns (from cache if available)
    auto patterns = getPatterns(msg);
    
    if (patterns.empty()) {
        RCLCPP_WARN(this->get_logger(), "No patterns to process");
        return visualization_msgs::msg::MarkerArray();
    }
    
    // Create spatial markers if spatial information is available
    if (hasSpatialInfo(msg)) {
        return createSpatialMarkers(msg, patterns, voxel_size);
    } else {
        RCLCPP_WARN(this->get_logger(), "No spatial information available");
        return visualization_msgs::msg::MarkerArray();
    }
}

double DecompressedViewerNode::getVoxelSize(
    const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg) const 
{
    if (msg->voxel_size > 0) {
        return msg->voxel_size;
    }
    // Use default value as fallback
    RCLCPP_WARN_ONCE(this->get_logger(), 
                     "No voxel_size in message, using default value: 0.01");
    return 0.01;  // Default to 1cm
}

std::vector<Block3D<8>> DecompressedViewerNode::getPatterns(
    const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg) 
{
    size_t pattern_hash = calculatePatternHash(msg->dictionary_data);
    
    if (cached_pattern_hash_ == pattern_hash && !cached_patterns_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Using cached patterns (hash: %zu)", pattern_hash);
        return cached_patterns_;
    }
    
    // Decompress patterns
    auto decompress_start = std::chrono::high_resolution_clock::now();
    auto patterns = pattern_decompressor_->decompress(
        msg->dictionary_data,
        msg->num_patterns,
        msg->block_size
    );
    auto decompress_end = std::chrono::high_resolution_clock::now();
    auto decompress_time = std::chrono::duration_cast<std::chrono::microseconds>(decompress_end - decompress_start).count() / 1000.0;
    
    RCLCPP_DEBUG(this->get_logger(), "[Decompressed] Decompress %u patterns: %.2f ms", 
                 msg->num_patterns, decompress_time);
    
    // Update cache
    cached_patterns_ = patterns;
    cached_pattern_hash_ = pattern_hash;
    
    return patterns;
}

bool DecompressedViewerNode::hasSpatialInfo(
    const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg) const 
{
    return (msg->blocks_x > 0 &&
            msg->blocks_y > 0 &&
            msg->blocks_z > 0 &&
            !msg->block_indices_data.empty());
}

visualization_msgs::msg::MarkerArray DecompressedViewerNode::createSpatialMarkers(
    const pointcloud_compressor::msg::PatternDictionary::SharedPtr msg,
    const std::vector<Block3D<8>>& patterns,
    double voxel_size) 
{
    auto blocks_dims = std::make_tuple(msg->blocks_x, msg->blocks_y, msg->blocks_z);
    auto grid_origin = std::make_tuple(
        msg->voxel_grid_origin.x,
        msg->voxel_grid_origin.y,
        msg->voxel_grid_origin.z
    );
    
    // Extract block indices from the message based on index_bit_size
    std::vector<uint16_t> block_indices;
    if (msg->index_bit_size == 8) {
        // 8-bit encoding: each byte is one index
        block_indices.reserve(msg->block_indices_data.size());
        for (uint8_t byte : msg->block_indices_data) {
            block_indices.push_back(static_cast<uint16_t>(byte));
        }
    } else if (msg->index_bit_size == 16) {
        // 16-bit encoding: two bytes form one index (little-endian)
        size_t num_indices = msg->block_indices_data.size() / 2;
        block_indices.reserve(num_indices);
        
        for (size_t i = 0; i < msg->block_indices_data.size(); i += 2) {
            uint16_t idx = static_cast<uint16_t>(msg->block_indices_data[i]) |
                          (static_cast<uint16_t>(msg->block_indices_data[i + 1]) << 8);
            block_indices.push_back(idx);
        }
    }
    
    return pattern_visualizer_->createSpatialPatternMarkers(
        patterns,
        block_indices,
        blocks_dims,
        voxel_size,
        msg->block_size,
        grid_origin
    );
}

void DecompressedViewerNode::logMarkerInfo(const visualization_msgs::msg::MarkerArray& marker_array) {
    // Count marker types
    std::map<int, int> marker_types;
    for (const auto& marker : marker_array.markers) {
        marker_types[marker.type]++;
    }
    
    RCLCPP_INFO(this->get_logger(), "Published %zu markers", marker_array.markers.size());
    
    if (!marker_types.empty()) {
        std::stringstream ss;
        ss << "Marker types: ";
        for (const auto& [type, count] : marker_types) {
            ss << type << ":" << count << " ";
        }
        RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
    }
}

size_t DecompressedViewerNode::calculatePatternHash(const std::vector<uint8_t>& data) const {
    std::hash<std::string> hasher;
    return hasher(std::string(data.begin(), data.end()));
}

} // namespace decompressed