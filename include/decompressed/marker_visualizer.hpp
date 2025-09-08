#pragma once

#include <vector>
#include <memory>
#include <string>
#include <tuple>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>

#include "decompressed/pattern_dictionary_decompressor.hpp"

namespace decompressed {

/**
 * @brief Creates MarkerArray visualizations for pattern dictionary data
 */
class MarkerVisualizer {
public:
    using Block = Block3D<8>;  // Default to 8x8x8 blocks
    using Color = std::tuple<float, float, float, float>;  // RGBA
    
    /**
     * @brief Constructor
     * @param frame_id Frame ID for the markers
     */
    explicit MarkerVisualizer(const std::string& frame_id = "map");
    
    ~MarkerVisualizer() = default;
    
    /**
     * @brief Create MarkerArray with patterns positioned at their actual 3D locations
     * @param patterns List of 3D boolean blocks representing patterns
     * @param block_indices List mapping block positions to pattern indices
     * @param blocks_dims Number of blocks in each dimension (x, y, z)
     * @param voxel_size Size of each voxel in meters
     * @param block_size Size of each block in voxels
     * @param grid_origin Origin of the voxel grid in world coordinates
     * @return MarkerArray message containing spatially positioned visualization markers
     */
    visualization_msgs::msg::MarkerArray createSpatialPatternMarkers(
        const std::vector<Block>& patterns,
        const std::vector<uint16_t>& block_indices,
        const std::tuple<uint32_t, uint32_t, uint32_t>& blocks_dims,
        double voxel_size,
        uint32_t block_size,
        const std::tuple<double, double, double>& grid_origin = std::make_tuple(0.0, 0.0, 0.0));
    
    /**
     * @brief Create MarkerArray from a list of pattern blocks with spacing
     * @param patterns List of 3D boolean blocks representing patterns
     * @param pattern_spacing Spacing between patterns in meters
     * @param voxel_size Size of each voxel in meters
     * @return MarkerArray message containing visualization markers
     */
    visualization_msgs::msg::MarkerArray createPatternMarkers(
        const std::vector<Block>& patterns,
        double pattern_spacing = 3.0,
        double voxel_size = 0.1);
    
    /**
     * @brief Create MarkerArray to clear all markers
     * @return MarkerArray with DELETE_ALL action
     */
    visualization_msgs::msg::MarkerArray clearMarkers();
    
    /**
     * @brief Create information text markers for pattern display
     * @param info_text Text to display
     * @param position Position for the text marker
     * @return MarkerArray containing text marker
     */
    visualization_msgs::msg::MarkerArray createInfoMarkers(
        const std::string& info_text,
        const std::tuple<double, double, double>& position = std::make_tuple(0.0, 0.0, 2.0));
    
private:
    std::string frame_id_;
    
    /**
     * @brief Calculate world positions for occupied voxels in a pattern
     * @param pattern 3D boolean block
     * @param voxel_size Size of each voxel
     * @param offset Offset for this pattern
     * @return List of (x, y, z) positions for occupied voxels
     */
    std::vector<std::tuple<double, double, double>> createVoxelPositions(
        const Block& pattern,
        double voxel_size,
        const std::tuple<double, double, double>& offset);
    
    /**
     * @brief Validate that a pattern is valid
     * @param pattern Block to validate
     * @return True if valid, False otherwise
     */
    bool validatePattern(const Block& pattern);
    
    /**
     * @brief Generate default color for a pattern based on its index
     * @param pattern_index Index of the pattern
     * @return (r, g, b, a) color tuple
     */
    Color getDefaultColor(int pattern_index);
};

} // namespace decompressed