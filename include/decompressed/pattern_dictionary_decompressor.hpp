#pragma once

#include <vector>
#include <cstdint>
#include <array>
#include <memory>

namespace decompressed {

/**
 * @brief 3D Block structure for pattern data
 */
template<size_t BlockSize>
class Block3D {
public:
    static constexpr size_t SIZE = BlockSize;
    static constexpr size_t TOTAL_VOXELS = BlockSize * BlockSize * BlockSize;
    
    Block3D() : data_{} {}
    
    bool get(size_t x, size_t y, size_t z) const {
        if (x >= SIZE || y >= SIZE || z >= SIZE) {
            return false;
        }
        return data_[z * SIZE * SIZE + y * SIZE + x];
    }
    
    void set(size_t x, size_t y, size_t z, bool value) {
        if (x >= SIZE || y >= SIZE || z >= SIZE) {
            return;
        }
        data_[z * SIZE * SIZE + y * SIZE + x] = value;
    }
    
    const std::array<bool, TOTAL_VOXELS>& getData() const { return data_; }
    std::array<bool, TOTAL_VOXELS>& getData() { return data_; }

private:
    std::array<bool, TOTAL_VOXELS> data_;
};

/**
 * @brief Decompressor for pattern dictionary
 */
class PatternDictionaryDecompressor {
public:
    using Block = Block3D<8>;  // Default to 8x8x8 blocks
    
    PatternDictionaryDecompressor() = default;
    ~PatternDictionaryDecompressor() = default;
    
    /**
     * @brief Decompress pattern dictionary from raw data
     * @param dictionary_data Raw dictionary data
     * @param num_patterns Number of patterns in dictionary
     * @param block_size Size of each block dimension
     * @return Vector of decompressed blocks
     */
    std::vector<Block> decompress(
        const std::vector<uint8_t>& dictionary_data,
        uint32_t num_patterns,
        uint32_t block_size = 8
    );
    
    /**
     * @brief Calculate CRC32 checksum for validation
     * @param data Input data
     * @return CRC32 checksum
     */
    uint32_t calculateCRC32(const std::vector<uint8_t>& data) const;
    
    /**
     * @brief Verify checksum matches expected value
     * @param data Input data
     * @param expected_checksum Expected CRC32 value
     * @return True if checksum matches
     */
    bool verifyChecksum(const std::vector<uint8_t>& data, uint32_t expected_checksum) const;

private:
    /**
     * @brief Decompress a single block from raw data
     * @param data Raw block data
     * @param block_size Size of block dimension
     * @return Decompressed block
     */
    Block decompressBlock(const std::vector<uint8_t>& data, uint32_t block_size);
    
    /**
     * @brief Convert linear index to 3D coordinates
     * @param index Linear index
     * @param block_size Size of block dimension
     * @param[out] x X coordinate
     * @param[out] y Y coordinate  
     * @param[out] z Z coordinate
     */
    void indexToCoordinates(size_t index, uint32_t block_size, 
                           size_t& x, size_t& y, size_t& z) const;
};

} // namespace decompressed