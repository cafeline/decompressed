#include <gtest/gtest.h>
#include "decompressed/pattern_dictionary_decompressor.hpp"
#include <bitset>

using namespace decompressed;

class PatternDictionaryDecompressorTest : public ::testing::Test {
protected:
    PatternDictionaryDecompressor decompressor;
    
    // Helper function to create test pattern data
    std::vector<uint8_t> createTestPattern(size_t block_size) {
        size_t total_voxels = block_size * block_size * block_size;
        size_t bytes_needed = (total_voxels + 7) / 8;  // Round up to nearest byte
        std::vector<uint8_t> data(bytes_needed, 0);
        
        // Set some specific bits for testing
        // Pattern: set voxels at (0,0,0), (1,1,1), (2,2,2)
        data[0] = 0b00000001;  // (0,0,0)
        if (block_size >= 2) {
            size_t idx = 1 * block_size * block_size + 1 * block_size + 1;  // (1,1,1)
            data[idx / 8] |= (1 << (idx % 8));
        }
        if (block_size >= 3) {
            size_t idx = 2 * block_size * block_size + 2 * block_size + 2;  // (2,2,2)
            data[idx / 8] |= (1 << (idx % 8));
        }
        
        return data;
    }
};

TEST_F(PatternDictionaryDecompressorTest, TestBlock3DGetSet) {
    Block3D<8> block;
    
    // Test initial state (all false)
    EXPECT_FALSE(block.get(0, 0, 0));
    EXPECT_FALSE(block.get(7, 7, 7));
    
    // Test set and get
    block.set(0, 0, 0, true);
    EXPECT_TRUE(block.get(0, 0, 0));
    
    block.set(7, 7, 7, true);
    EXPECT_TRUE(block.get(7, 7, 7));
    
    block.set(3, 4, 5, true);
    EXPECT_TRUE(block.get(3, 4, 5));
    
    // Test boundary conditions
    block.set(8, 8, 8, true);  // Out of bounds, should not crash
    EXPECT_FALSE(block.get(8, 8, 8));  // Should return false for out of bounds
}

TEST_F(PatternDictionaryDecompressorTest, TestDecompressSinglePattern) {
    // Create test data for one 8x8x8 block
    std::vector<uint8_t> test_data = createTestPattern(8);
    
    // Decompress
    auto blocks = decompressor.decompress(test_data, 1, 8);
    
    // Verify
    ASSERT_EQ(blocks.size(), 1);
    
    // Check specific voxels
    EXPECT_TRUE(blocks[0].get(0, 0, 0));  // Should be set
    EXPECT_FALSE(blocks[0].get(0, 0, 1)); // Should not be set
    
    if (8 >= 2) {
        EXPECT_TRUE(blocks[0].get(1, 1, 1));  // Should be set
    }
    if (8 >= 3) {
        EXPECT_TRUE(blocks[0].get(2, 2, 2));  // Should be set
    }
}

TEST_F(PatternDictionaryDecompressorTest, TestDecompressMultiplePatterns) {
    // Create test data for 3 patterns
    std::vector<uint8_t> pattern1 = createTestPattern(8);
    std::vector<uint8_t> pattern2 = createTestPattern(8);
    std::vector<uint8_t> pattern3 = createTestPattern(8);
    
    // Modify patterns to be different
    pattern2[0] = 0b00000011;  // Different pattern
    pattern3[0] = 0b00000111;  // Another different pattern
    
    // Combine patterns
    std::vector<uint8_t> combined_data;
    combined_data.insert(combined_data.end(), pattern1.begin(), pattern1.end());
    combined_data.insert(combined_data.end(), pattern2.begin(), pattern2.end());
    combined_data.insert(combined_data.end(), pattern3.begin(), pattern3.end());
    
    // Decompress
    auto blocks = decompressor.decompress(combined_data, 3, 8);
    
    // Verify
    ASSERT_EQ(blocks.size(), 3);
    
    // Check first pattern
    EXPECT_TRUE(blocks[0].get(0, 0, 0));
    EXPECT_FALSE(blocks[0].get(1, 0, 0));
    
    // Check second pattern  
    EXPECT_TRUE(blocks[1].get(0, 0, 0));
    EXPECT_TRUE(blocks[1].get(1, 0, 0));  // Should be set due to 0b00000011
    
    // Check third pattern
    EXPECT_TRUE(blocks[2].get(0, 0, 0));
    EXPECT_TRUE(blocks[2].get(1, 0, 0));
    EXPECT_TRUE(blocks[2].get(2, 0, 0));  // Should be set due to 0b00000111
}

TEST_F(PatternDictionaryDecompressorTest, TestCRC32Calculation) {
    std::vector<uint8_t> test_data = {0x12, 0x34, 0x56, 0x78};
    
    uint32_t crc1 = decompressor.calculateCRC32(test_data);
    uint32_t crc2 = decompressor.calculateCRC32(test_data);
    
    // Same data should produce same CRC
    EXPECT_EQ(crc1, crc2);
    
    // Different data should produce different CRC
    test_data[0] = 0x13;
    uint32_t crc3 = decompressor.calculateCRC32(test_data);
    EXPECT_NE(crc1, crc3);
}

TEST_F(PatternDictionaryDecompressorTest, TestChecksumVerification) {
    std::vector<uint8_t> test_data = {0x12, 0x34, 0x56, 0x78};
    
    uint32_t checksum = decompressor.calculateCRC32(test_data);
    
    // Correct checksum should pass
    EXPECT_TRUE(decompressor.verifyChecksum(test_data, checksum));
    
    // Incorrect checksum should fail
    EXPECT_FALSE(decompressor.verifyChecksum(test_data, checksum + 1));
    
    // Modified data should fail
    test_data[0] = 0x13;
    EXPECT_FALSE(decompressor.verifyChecksum(test_data, checksum));
}

TEST_F(PatternDictionaryDecompressorTest, TestEmptyData) {
    std::vector<uint8_t> empty_data;
    
    // Should handle empty data gracefully
    auto blocks = decompressor.decompress(empty_data, 0, 8);
    EXPECT_EQ(blocks.size(), 0);
}

TEST_F(PatternDictionaryDecompressorTest, TestCoordinateConversion) {
    // Test that coordinate conversion matches expected order
    // This is critical for matching the compression side
    Block3D<8> block;
    
    // Set voxels in specific order
    for (size_t z = 0; z < 8; ++z) {
        for (size_t y = 0; y < 8; ++y) {
            for (size_t x = 0; x < 8; ++x) {
                size_t linear_idx = z * 64 + y * 8 + x;
                if (linear_idx < 10) {  // Set first 10 voxels
                    block.set(x, y, z, true);
                }
            }
        }
    }
    
    // Verify first 10 voxels are set in correct positions
    EXPECT_TRUE(block.get(0, 0, 0));  // idx 0
    EXPECT_TRUE(block.get(1, 0, 0));  // idx 1
    EXPECT_TRUE(block.get(2, 0, 0));  // idx 2
    EXPECT_TRUE(block.get(3, 0, 0));  // idx 3
    EXPECT_TRUE(block.get(4, 0, 0));  // idx 4
    EXPECT_TRUE(block.get(5, 0, 0));  // idx 5
    EXPECT_TRUE(block.get(6, 0, 0));  // idx 6
    EXPECT_TRUE(block.get(7, 0, 0));  // idx 7
    EXPECT_TRUE(block.get(0, 1, 0));  // idx 8
    EXPECT_TRUE(block.get(1, 1, 0));  // idx 9
    
    // Verify voxel 10 is not set
    EXPECT_FALSE(block.get(2, 1, 0));  // idx 10
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}