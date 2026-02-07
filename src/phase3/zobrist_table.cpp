#include "openpanelcam/phase3/zobrist_table.h"

namespace openpanelcam {
namespace phase3 {

ZobristTable::ZobristTable(uint64_t seed) {
    std::mt19937_64 rng;
    if (seed == 0) {
        std::random_device rd;
        rng.seed(rd());
    } else {
        rng.seed(seed);
    }

    for (int i = 0; i < MAX_BENDS; i++) {
        m_bendHashes[i] = rng();
    }

    for (int i = 0; i < NUM_ORIENTATIONS; i++) {
        m_orientationHashes[i] = rng();
    }

    for (int i = 0; i < ABA_CONFIG_BUCKETS; i++) {
        m_abaHashes[i] = rng();
    }
}

uint64_t ZobristTable::hash(const SearchState& state) const {
    uint64_t h = 0;

    for (int i = 0; i < MAX_BENDS; i++) {
        if (state.isBent(i)) {
            h ^= m_bendHashes[i];
        }
    }

    h ^= m_orientationHashes[static_cast<uint8_t>(state.orientation)];
    h ^= m_abaHashes[abaConfigToBucket(state.abaConfig)];

    return h;
}

uint64_t ZobristTable::updateBend(uint64_t oldHash, int bendId) const {
    if (bendId < 0 || bendId >= MAX_BENDS) return oldHash;
    return oldHash ^ m_bendHashes[bendId];
}

uint64_t ZobristTable::updateOrientation(uint64_t oldHash,
                                         Orientation oldOri,
                                         Orientation newOri) const {
    uint64_t h = oldHash;
    h ^= m_orientationHashes[static_cast<uint8_t>(oldOri)];
    h ^= m_orientationHashes[static_cast<uint8_t>(newOri)];
    return h;
}

uint64_t ZobristTable::updateAba(uint64_t oldHash,
                                 uint16_t oldConfig,
                                 uint16_t newConfig) const {
    uint64_t h = oldHash;
    h ^= m_abaHashes[abaConfigToBucket(oldConfig)];
    h ^= m_abaHashes[abaConfigToBucket(newConfig)];
    return h;
}

uint16_t ZobristTable::abaConfigToBucket(uint16_t config) const {
    return config % ABA_CONFIG_BUCKETS;
}

} // namespace phase3
} // namespace openpanelcam
