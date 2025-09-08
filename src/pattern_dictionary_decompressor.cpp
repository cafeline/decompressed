#include "decompressed/pattern_dictionary_decompressor.hpp"
#include <cstring>
#include <stdexcept>

namespace decompressed {

std::vector<PatternDictionaryDecompressor::Block> 
PatternDictionaryDecompressor::decompress(
    const std::vector<uint8_t>& dictionary_data,
    uint32_t num_patterns,
    uint32_t block_size) 
{
    std::vector<Block> blocks;
    blocks.reserve(num_patterns);
    
    if (num_patterns == 0 || dictionary_data.empty()) {
        return blocks;
    }
    
    // Calculate size of each pattern in bytes
    size_t voxels_per_block = block_size * block_size * block_size;
    size_t bytes_per_pattern = (voxels_per_block + 7) / 8;  // Round up to nearest byte
    
    // Check if we have enough data
    if (dictionary_data.size() < num_patterns * bytes_per_pattern) {
        throw std::runtime_error("Insufficient data for number of patterns");
    }
    
    // Decompress each pattern
    for (uint32_t i = 0; i < num_patterns; ++i) {
        size_t offset = i * bytes_per_pattern;
        std::vector<uint8_t> pattern_data(
            dictionary_data.begin() + offset,
            dictionary_data.begin() + offset + bytes_per_pattern
        );
        
        blocks.push_back(decompressBlock(pattern_data, block_size));
    }
    
    return blocks;
}

PatternDictionaryDecompressor::Block 
PatternDictionaryDecompressor::decompressBlock(
    const std::vector<uint8_t>& data, 
    uint32_t block_size) 
{
    Block block;
    
    // Process each bit in the data
    size_t bit_index = 0;
    for (size_t z = 0; z < block_size; ++z) {
        for (size_t y = 0; y < block_size; ++y) {
            for (size_t x = 0; x < block_size; ++x) {
                if (bit_index / 8 >= data.size()) {
                    break;
                }
                
                // Extract bit value
                uint8_t byte = data[bit_index / 8];
                uint8_t bit = (byte >> (bit_index % 8)) & 1;
                
                // Set voxel
                block.set(x, y, z, bit == 1);
                
                bit_index++;
            }
        }
    }
    
    return block;
}

void PatternDictionaryDecompressor::indexToCoordinates(
    size_t index, 
    uint32_t block_size,
    size_t& x, 
    size_t& y, 
    size_t& z) const 
{
    // Convert linear index to 3D coordinates
    // Using same order as compression: x varies fastest, then y, then z
    x = index % block_size;
    y = (index / block_size) % block_size;
    z = index / (block_size * block_size);
}

uint32_t PatternDictionaryDecompressor::calculateCRC32(
    const std::vector<uint8_t>& data) const 
{
    // Simple CRC32 implementation
    // Note: This is a basic implementation for testing
    // In production, use a proper CRC32 library
    
    uint32_t crc = 0xFFFFFFFF;
    
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return ~crc;
}

bool PatternDictionaryDecompressor::verifyChecksum(
    const std::vector<uint8_t>& data, 
    uint32_t expected_checksum) const 
{
    return calculateCRC32(data) == expected_checksum;
}

} // namespace decompressed