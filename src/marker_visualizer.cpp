#include "decompressed/marker_visualizer.hpp"
#include <rclcpp/logging.hpp>

namespace decompressed {

MarkerVisualizer::MarkerVisualizer(const std::string& frame_id)
    : frame_id_(frame_id) {
}

visualization_msgs::msg::MarkerArray MarkerVisualizer::createSpatialPatternMarkers(
    const std::vector<Block>& patterns,
    const std::vector<uint16_t>& block_indices,
    const std::tuple<uint32_t, uint32_t, uint32_t>& blocks_dims,
    double voxel_size,
    uint32_t block_size,
    const std::tuple<double, double, double>& grid_origin) 
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    if (patterns.empty() || block_indices.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("MarkerVisualizer"),
            "create_spatial_pattern_markers - patterns: %zu, block_indices: %zu",
            patterns.size(), block_indices.size());
        return marker_array;
    }
    
    auto [blocks_x, blocks_y, blocks_z] = blocks_dims;
    double block_size_meters = block_size * voxel_size;
    
    // Create a marker for each block
    int marker_id = 0;
    int skipped_blocks = 0;
    int empty_patterns = 0;
    int total_voxels = 0;
    
    RCLCPP_DEBUG(rclcpp::get_logger("MarkerVisualizer"),
        "Creating spatial markers for %zu blocks with %zu patterns",
        block_indices.size(), patterns.size());
    
    for (size_t block_idx = 0; block_idx < block_indices.size(); ++block_idx) {
        uint16_t pattern_idx = block_indices[block_idx];
        
        if (pattern_idx >= patterns.size()) {
            skipped_blocks++;
            RCLCPP_DEBUG(rclcpp::get_logger("MarkerVisualizer"),
                "Block %zu skipped - pattern_idx %d >= %zu",
                block_idx, pattern_idx, patterns.size());
            continue;
        }
        
        const Block& pattern = patterns[pattern_idx];
        if (!validatePattern(pattern)) {
            continue;
        }
        
        // Calculate block position in 3D grid
        uint32_t bz = block_idx / (blocks_x * blocks_y);
        uint32_t by = (block_idx % (blocks_x * blocks_y)) / blocks_x;
        uint32_t bx = block_idx % blocks_x;
        
        // Calculate world position for this block
        auto block_world_pos = std::make_tuple(
            std::get<0>(grid_origin) + bx * block_size_meters,
            std::get<1>(grid_origin) + by * block_size_meters,
            std::get<2>(grid_origin) + bz * block_size_meters
        );
        
        // Get voxel positions for this pattern (relative to block origin)
        auto voxel_positions = createVoxelPositions(pattern, voxel_size, 
                                                   std::make_tuple(0.0, 0.0, 0.0));
        
        if (voxel_positions.empty()) {
            empty_patterns++;
            continue;
        }
        
        total_voxels += voxel_positions.size();
        
        // Create individual CUBE markers for each voxel
        for (const auto& voxel_pos : voxel_positions) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = rclcpp::Time();
            marker.id = marker_id;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set marker position (block position + voxel position)
            marker.pose.position.x = std::get<0>(block_world_pos) + std::get<0>(voxel_pos);
            marker.pose.position.y = std::get<1>(block_world_pos) + std::get<1>(voxel_pos);
            marker.pose.position.z = std::get<2>(block_world_pos) + std::get<2>(voxel_pos);
            marker.pose.orientation.w = 1.0;
            
            // Set marker scale (slightly smaller than voxel size for visibility)
            marker.scale.x = voxel_size * 0.9;
            marker.scale.y = voxel_size * 0.9;
            marker.scale.z = voxel_size * 0.9;
            
            // Set marker color to blue (all markers same color)
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.5;  // Semi-transparent
            
            marker_array.markers.push_back(marker);
            marker_id++;
        }
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("MarkerVisualizer"),
        "Created %d markers with %d total voxels", marker_id, total_voxels);
    RCLCPP_DEBUG(rclcpp::get_logger("MarkerVisualizer"),
        "Skipped %d blocks (invalid indices), %d blocks (empty patterns)",
        skipped_blocks, empty_patterns);
    
    if (skipped_blocks > 0) {
        RCLCPP_WARN(rclcpp::get_logger("MarkerVisualizer"),
            "Skipped %d blocks due to invalid pattern indices", skipped_blocks);
    }
    
    return marker_array;
}

visualization_msgs::msg::MarkerArray MarkerVisualizer::createPatternMarkers(
    const std::vector<Block>& patterns,
    double pattern_spacing,
    double voxel_size) 
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    if (patterns.empty()) {
        return marker_array;
    }
    
    int marker_id = 0;
    for (size_t i = 0; i < patterns.size(); ++i) {
        const Block& pattern = patterns[i];
        if (!validatePattern(pattern)) {
            continue;
        }
        
        // Calculate pattern offset
        auto pattern_offset = std::make_tuple(i * pattern_spacing, 0.0, 0.0);
        
        // Get voxel positions for this pattern (relative to pattern origin)
        auto voxel_positions = createVoxelPositions(pattern, voxel_size, 
                                                   std::make_tuple(0.0, 0.0, 0.0));
        
        if (voxel_positions.empty()) {
            continue;
        }
        
        // Create individual CUBE markers for each voxel
        for (const auto& voxel_pos : voxel_positions) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = rclcpp::Time();
            marker.id = marker_id;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set marker position (pattern offset + voxel position)
            marker.pose.position.x = std::get<0>(pattern_offset) + std::get<0>(voxel_pos);
            marker.pose.position.y = std::get<1>(pattern_offset) + std::get<1>(voxel_pos);
            marker.pose.position.z = std::get<2>(pattern_offset) + std::get<2>(voxel_pos);
            marker.pose.orientation.w = 1.0;
            
            // Set marker scale (slightly smaller than voxel size for visibility)
            marker.scale.x = voxel_size * 0.9;
            marker.scale.y = voxel_size * 0.9;
            marker.scale.z = voxel_size * 0.9;
            
            // Set marker color to blue (all markers same color)
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 0.5;  // Semi-transparent
            
            marker_array.markers.push_back(marker);
            marker_id++;
        }
    }
    
    return marker_array;
}

visualization_msgs::msg::MarkerArray MarkerVisualizer::clearMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = frame_id_;
    clear_marker.header.stamp = rclcpp::Time();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    
    marker_array.markers.push_back(clear_marker);
    
    return marker_array;
}

visualization_msgs::msg::MarkerArray MarkerVisualizer::createInfoMarkers(
    const std::string& info_text,
    const std::tuple<double, double, double>& position) 
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id_;
    text_marker.header.stamp = rclcpp::Time();
    text_marker.id = 999999;  // High ID to avoid conflicts
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    text_marker.pose.position.x = std::get<0>(position);
    text_marker.pose.position.y = std::get<1>(position);
    text_marker.pose.position.z = std::get<2>(position);
    text_marker.pose.orientation.w = 1.0;
    
    text_marker.scale.z = 0.2;  // Text height
    
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    text_marker.text = info_text;
    
    marker_array.markers.push_back(text_marker);
    
    return marker_array;
}

std::vector<std::tuple<double, double, double>> MarkerVisualizer::createVoxelPositions(
    const Block& pattern,
    double voxel_size,
    const std::tuple<double, double, double>& offset) 
{
    std::vector<std::tuple<double, double, double>> positions;
    
    // Iterate in z->y->x order to match C++ side iteration order
    // This ensures markers are created in the same order as occupied_voxel_markers
    for (size_t z_idx = 0; z_idx < Block::SIZE; ++z_idx) {
        for (size_t y_idx = 0; y_idx < Block::SIZE; ++y_idx) {
            for (size_t x_idx = 0; x_idx < Block::SIZE; ++x_idx) {
                if (pattern.get(x_idx, y_idx, z_idx)) {
                    // Calculate world position
                    double x = std::get<0>(offset) + (x_idx + 0.5) * voxel_size;
                    double y = std::get<1>(offset) + (y_idx + 0.5) * voxel_size;
                    double z = std::get<2>(offset) + (z_idx + 0.5) * voxel_size;
                    positions.push_back(std::make_tuple(x, y, z));
                }
            }
        }
    }
    
    return positions;
}

bool MarkerVisualizer::validatePattern(const Block& pattern) {
    // For Block3D, validation is always true as it's a fixed-size structure
    // We could check if any voxel is set if needed
    return true;
}

MarkerVisualizer::Color MarkerVisualizer::getDefaultColor(int pattern_index) {
    // Define a set of distinct colors
    std::vector<Color> default_colors = {
        std::make_tuple(0.0f, 1.0f, 0.0f, 1.0f),  // Green
        std::make_tuple(0.0f, 0.0f, 1.0f, 1.0f),  // Blue
        std::make_tuple(1.0f, 1.0f, 0.0f, 1.0f),  // Yellow
        std::make_tuple(1.0f, 0.0f, 1.0f, 1.0f),  // Magenta
        std::make_tuple(0.0f, 1.0f, 1.0f, 1.0f),  // Cyan
        std::make_tuple(1.0f, 0.5f, 0.0f, 1.0f),  // Orange
        std::make_tuple(0.5f, 0.0f, 1.0f, 1.0f),  // Purple
        std::make_tuple(0.0f, 0.5f, 0.0f, 1.0f),  // Dark Green
    };
    
    return default_colors[pattern_index % default_colors.size()];
}

} // namespace decompressed