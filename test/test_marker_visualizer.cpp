#include <gtest/gtest.h>
#include "decompressed/marker_visualizer.hpp"
#include <cmath>

using namespace decompressed;

class MarkerVisualizerTest : public ::testing::Test {
protected:
    void SetUp() override {
        visualizer = std::make_unique<MarkerVisualizer>("test_frame");
    }
    
    std::unique_ptr<MarkerVisualizer> visualizer;
    
    // Helper function to create a test block with specific voxels set
    Block3D<8> createTestBlock(const std::vector<std::tuple<size_t, size_t, size_t>>& voxels) {
        Block3D<8> block;
        for (const auto& [x, y, z] : voxels) {
            block.set(x, y, z, true);
        }
        return block;
    }
    
    // Helper function to count markers in a MarkerArray
    size_t countMarkers(const visualization_msgs::msg::MarkerArray& array) {
        size_t count = 0;
        for (const auto& marker : array.markers) {
            if (marker.action != visualization_msgs::msg::Marker::DELETEALL) {
                count++;
            }
        }
        return count;
    }
    
    // Helper function to find a marker at specific position (with tolerance)
    bool hasMarkerAt(const visualization_msgs::msg::MarkerArray& array, 
                     double x, double y, double z, 
                     double tolerance = 1e-6) {
        for (const auto& marker : array.markers) {
            if (std::abs(marker.pose.position.x - x) < tolerance &&
                std::abs(marker.pose.position.y - y) < tolerance &&
                std::abs(marker.pose.position.z - z) < tolerance) {
                return true;
            }
        }
        return false;
    }
};

TEST_F(MarkerVisualizerTest, TestClearMarkers) {
    auto clear_array = visualizer->clearMarkers();
    
    ASSERT_EQ(clear_array.markers.size(), 1);
    EXPECT_EQ(clear_array.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
    EXPECT_EQ(clear_array.markers[0].header.frame_id, "test_frame");
}

TEST_F(MarkerVisualizerTest, TestCreateInfoMarkers) {
    std::string info_text = "Test Info";
    auto info_array = visualizer->createInfoMarkers(info_text, std::make_tuple(1.0, 2.0, 3.0));
    
    ASSERT_EQ(info_array.markers.size(), 1);
    const auto& marker = info_array.markers[0];
    
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    EXPECT_EQ(marker.text, info_text);
    EXPECT_DOUBLE_EQ(marker.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(marker.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(marker.pose.position.z, 3.0);
    EXPECT_EQ(marker.header.frame_id, "test_frame");
}

TEST_F(MarkerVisualizerTest, TestCreatePatternMarkersEmpty) {
    std::vector<Block3D<8>> patterns;
    
    auto marker_array = visualizer->createPatternMarkers(patterns, 3.0, 0.1);
    
    EXPECT_EQ(marker_array.markers.size(), 0);
}

TEST_F(MarkerVisualizerTest, TestCreatePatternMarkersSingle) {
    // Create a block with one voxel at (0,0,0)
    std::vector<Block3D<8>> patterns;
    patterns.push_back(createTestBlock({{0, 0, 0}}));
    
    double voxel_size = 0.1;
    auto marker_array = visualizer->createPatternMarkers(patterns, 3.0, voxel_size);
    
    // Should create one marker
    ASSERT_EQ(marker_array.markers.size(), 1);
    
    const auto& marker = marker_array.markers[0];
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::CUBE);
    EXPECT_EQ(marker.header.frame_id, "test_frame");
    
    // Position should be at center of voxel (0,0,0) -> (0.05, 0.05, 0.05)
    EXPECT_DOUBLE_EQ(marker.pose.position.x, 0.05);
    EXPECT_DOUBLE_EQ(marker.pose.position.y, 0.05);
    EXPECT_DOUBLE_EQ(marker.pose.position.z, 0.05);
    
    // Scale should be 90% of voxel size
    EXPECT_DOUBLE_EQ(marker.scale.x, voxel_size * 0.9);
    EXPECT_DOUBLE_EQ(marker.scale.y, voxel_size * 0.9);
    EXPECT_DOUBLE_EQ(marker.scale.z, voxel_size * 0.9);
    
    // Color should be blue with 0.5 alpha
    EXPECT_FLOAT_EQ(marker.color.r, 0.0f);
    EXPECT_FLOAT_EQ(marker.color.g, 0.0f);
    EXPECT_FLOAT_EQ(marker.color.b, 1.0f);
    EXPECT_FLOAT_EQ(marker.color.a, 0.5f);
}

TEST_F(MarkerVisualizerTest, TestCreatePatternMarkersMultiple) {
    // Create two patterns with different voxels
    std::vector<Block3D<8>> patterns;
    patterns.push_back(createTestBlock({{0, 0, 0}, {1, 1, 1}}));  // Pattern 0: 2 voxels
    patterns.push_back(createTestBlock({{2, 2, 2}}));              // Pattern 1: 1 voxel
    
    double voxel_size = 0.1;
    double pattern_spacing = 3.0;
    auto marker_array = visualizer->createPatternMarkers(patterns, pattern_spacing, voxel_size);
    
    // Should create 3 markers total (2 + 1)
    ASSERT_EQ(marker_array.markers.size(), 3);
    
    // Check first pattern's markers
    // First voxel at (0,0,0) in pattern 0
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.05, 0.05, 0.05));
    
    // Second voxel at (1,1,1) in pattern 0
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.15, 0.15, 0.15));
    
    // Third voxel at (2,2,2) in pattern 1, offset by pattern_spacing
    EXPECT_TRUE(hasMarkerAt(marker_array, pattern_spacing + 0.25, 0.25, 0.25));
}

TEST_F(MarkerVisualizerTest, TestCreateSpatialPatternMarkersEmpty) {
    std::vector<Block3D<8>> patterns;
    std::vector<uint8_t> block_indices;
    auto blocks_dims = std::make_tuple(2u, 2u, 2u);
    
    auto marker_array = visualizer->createSpatialPatternMarkers(
        patterns, block_indices, blocks_dims, 0.1, 8);
    
    EXPECT_EQ(marker_array.markers.size(), 0);
}

TEST_F(MarkerVisualizerTest, TestCreateSpatialPatternMarkersSingleBlock) {
    // Create one pattern with a voxel at (0,0,0)
    std::vector<Block3D<8>> patterns;
    patterns.push_back(createTestBlock({{0, 0, 0}}));
    
    // Map block 0 to pattern 0
    std::vector<uint8_t> block_indices = {0};
    
    // 1x1x1 block grid
    auto blocks_dims = std::make_tuple(1u, 1u, 1u);
    double voxel_size = 0.01;
    uint32_t block_size = 8;
    auto grid_origin = std::make_tuple(0.0, 0.0, 0.0);
    
    auto marker_array = visualizer->createSpatialPatternMarkers(
        patterns, block_indices, blocks_dims, voxel_size, block_size, grid_origin);
    
    // Should create 1 marker
    ASSERT_EQ(marker_array.markers.size(), 1);
    
    const auto& marker = marker_array.markers[0];
    
    // Position should be at center of voxel (0,0,0) in block (0,0,0)
    // Voxel center = grid_origin + block_offset + voxel_offset
    // = (0,0,0) + (0,0,0) + (0.005, 0.005, 0.005)
    EXPECT_DOUBLE_EQ(marker.pose.position.x, 0.005);
    EXPECT_DOUBLE_EQ(marker.pose.position.y, 0.005);
    EXPECT_DOUBLE_EQ(marker.pose.position.z, 0.005);
}

TEST_F(MarkerVisualizerTest, TestCreateSpatialPatternMarkersMultipleBlocks) {
    // Create two patterns
    std::vector<Block3D<8>> patterns;
    patterns.push_back(createTestBlock({{0, 0, 0}}));  // Pattern 0
    patterns.push_back(createTestBlock({{1, 1, 1}}));  // Pattern 1
    
    // Map blocks: block 0 -> pattern 0, block 1 -> pattern 1
    std::vector<uint8_t> block_indices = {0, 1};
    
    // 2x1x1 block grid (blocks arranged in x direction)
    auto blocks_dims = std::make_tuple(2u, 1u, 1u);
    double voxel_size = 0.01;
    uint32_t block_size = 8;
    auto grid_origin = std::make_tuple(0.0, 0.0, 0.0);
    
    auto marker_array = visualizer->createSpatialPatternMarkers(
        patterns, block_indices, blocks_dims, voxel_size, block_size, grid_origin);
    
    // Should create 2 markers
    ASSERT_EQ(marker_array.markers.size(), 2);
    
    // Block 0 at (0,0,0), voxel at (0,0,0)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.005, 0.005, 0.005));
    
    // Block 1 at (1,0,0) in block coords = (0.08, 0, 0) in world coords
    // Voxel at (1,1,1) within block = (0.015, 0.015, 0.015) relative to block
    // Total position = (0.08 + 0.015, 0.015, 0.015) = (0.095, 0.015, 0.015)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.095, 0.015, 0.015));
}

TEST_F(MarkerVisualizerTest, TestCreateSpatialPatternMarkers3DGrid) {
    // Create one pattern with voxel at (0,0,0)
    std::vector<Block3D<8>> patterns;
    patterns.push_back(createTestBlock({{0, 0, 0}}));
    
    // Use pattern 0 for all 8 blocks in 2x2x2 grid
    std::vector<uint8_t> block_indices = {0, 0, 0, 0, 0, 0, 0, 0};
    
    // 2x2x2 block grid
    auto blocks_dims = std::make_tuple(2u, 2u, 2u);
    double voxel_size = 0.01;
    uint32_t block_size = 8;
    auto grid_origin = std::make_tuple(0.0, 0.0, 0.0);
    
    auto marker_array = visualizer->createSpatialPatternMarkers(
        patterns, block_indices, blocks_dims, voxel_size, block_size, grid_origin);
    
    // Should create 8 markers (one per block)
    ASSERT_EQ(marker_array.markers.size(), 8);
    
    // Check positions for each block
    // Block indexing: idx = z * (blocks_x * blocks_y) + y * blocks_x + x
    double block_size_meters = block_size * voxel_size;  // 0.08
    
    // Block 0: (0,0,0)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.005, 0.005, 0.005));
    
    // Block 1: (1,0,0)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.085, 0.005, 0.005));
    
    // Block 2: (0,1,0)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.005, 0.085, 0.005));
    
    // Block 3: (1,1,0)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.085, 0.085, 0.005));
    
    // Block 4: (0,0,1)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.005, 0.005, 0.085));
    
    // Block 5: (1,0,1)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.085, 0.005, 0.085));
    
    // Block 6: (0,1,1)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.005, 0.085, 0.085));
    
    // Block 7: (1,1,1)
    EXPECT_TRUE(hasMarkerAt(marker_array, 0.085, 0.085, 0.085));
}

TEST_F(MarkerVisualizerTest, TestVoxelIterationOrder) {
    // Test that voxels are iterated in z->y->x order (matching C++ compressor)
    std::vector<Block3D<8>> patterns;
    Block3D<8> block;
    
    // Set voxels in specific order
    block.set(0, 0, 0, true);  // First
    block.set(1, 0, 0, true);  // Second
    block.set(0, 1, 0, true);  // Third
    block.set(0, 0, 1, true);  // Fourth
    
    patterns.push_back(block);
    
    double voxel_size = 0.1;
    auto marker_array = visualizer->createPatternMarkers(patterns, 3.0, voxel_size);
    
    // Should create 4 markers
    ASSERT_EQ(marker_array.markers.size(), 4);
    
    // Check that markers are created in z->y->x iteration order
    // This matches the order in the Python implementation
    EXPECT_EQ(marker_array.markers[0].id, 0);
    EXPECT_DOUBLE_EQ(marker_array.markers[0].pose.position.x, 0.05);  // (0,0,0)
    EXPECT_DOUBLE_EQ(marker_array.markers[0].pose.position.y, 0.05);
    EXPECT_DOUBLE_EQ(marker_array.markers[0].pose.position.z, 0.05);
    
    EXPECT_EQ(marker_array.markers[1].id, 1);
    EXPECT_DOUBLE_EQ(marker_array.markers[1].pose.position.x, 0.15);  // (1,0,0)
    EXPECT_DOUBLE_EQ(marker_array.markers[1].pose.position.y, 0.05);
    EXPECT_DOUBLE_EQ(marker_array.markers[1].pose.position.z, 0.05);
    
    EXPECT_EQ(marker_array.markers[2].id, 2);
    EXPECT_DOUBLE_EQ(marker_array.markers[2].pose.position.x, 0.05);  // (0,1,0)
    EXPECT_DOUBLE_EQ(marker_array.markers[2].pose.position.y, 0.15);
    EXPECT_DOUBLE_EQ(marker_array.markers[2].pose.position.z, 0.05);
    
    EXPECT_EQ(marker_array.markers[3].id, 3);
    EXPECT_DOUBLE_EQ(marker_array.markers[3].pose.position.x, 0.05);  // (0,0,1)
    EXPECT_DOUBLE_EQ(marker_array.markers[3].pose.position.y, 0.05);
    EXPECT_DOUBLE_EQ(marker_array.markers[3].pose.position.z, 0.15);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}