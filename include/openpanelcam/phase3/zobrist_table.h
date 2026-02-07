#pragma once

#include <cstdint>
#include <array>
#include <random>
#include "types.h"

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Zobrist hashing table for fast state deduplication
 *
 * Precomputes random 64-bit values for each state component.
 * Hash = XOR of all relevant component hashes.
 * Enables O(1) duplicate state detection in A* search.
 */
class ZobristTable {
public:
    static constexpr int MAX_BENDS = 32;
    static constexpr int NUM_ORIENTATIONS = 4;
    static constexpr int ABA_CONFIG_BUCKETS = 256;

    explicit ZobristTable(uint64_t seed = 0);

    uint64_t hash(const SearchState& state) const;

    uint64_t updateBend(uint64_t oldHash, int bendId) const;

    uint64_t updateOrientation(uint64_t oldHash,
                               Orientation oldOri,
                               Orientation newOri) const;

    uint64_t updateAba(uint64_t oldHash,
                       uint16_t oldConfig,
                       uint16_t newConfig) const;

private:
    std::array<uint64_t, MAX_BENDS> m_bendHashes;
    std::array<uint64_t, NUM_ORIENTATIONS> m_orientationHashes;
    std::array<uint64_t, ABA_CONFIG_BUCKETS> m_abaHashes;

    uint16_t abaConfigToBucket(uint16_t config) const;
};

} // namespace phase3
} // namespace openpanelcam
