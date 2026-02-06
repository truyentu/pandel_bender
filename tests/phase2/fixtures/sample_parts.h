#pragma once

/**
 * @file sample_parts.h
 * @brief Test fixtures for common sheet metal parts
 *
 * Provides realistic bend configurations for:
 * - L-bracket: Simple 2-bend right-angle bracket
 * - U-channel: 3-bend channel profile
 * - Box: 4-bend closed box configuration
 * - Complex panel: 7-bend panel with mixed angles
 *
 * Note: Bend positions are spread far enough apart (150mm+) to avoid
 * geometric precedence conflicts that would cause cycles.
 *
 * Task 35: Sample Data Fixtures
 */

#include "openpanelcam/phase2/phase1_mock.h"
#include <vector>

namespace openpanelcam {
namespace phase2 {
namespace fixtures {

/**
 * @brief Create L-bracket bend configuration
 *
 * Simple L-shaped bracket with two 90-degree bends:
 *   ________
 *  |        |  <- Bend 0 (horizontal flange)
 *  |
 *  |  <- Bend 1 (vertical flange)
 *  |
 *
 * @return Vector of 2 BendFeature objects
 */
inline std::vector<phase1::BendFeature> createLBracket() {
    std::vector<phase1::BendFeature> bends;

    phase1::BendFeature b0;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position = {0.0, 0.0, 0.0};
    b0.direction = {1.0, 0.0, 0.0};
    b0.normal = {0.0, 0.0, 1.0};
    bends.push_back(b0);

    phase1::BendFeature b1;
    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 80.0;
    b1.position = {0.0, 200.0, 0.0};  // Far apart to avoid conflicts
    b1.direction = {0.0, 1.0, 0.0};
    b1.normal = {0.0, 0.0, 1.0};
    bends.push_back(b1);

    return bends;
}

/**
 * @brief Create U-channel bend configuration
 *
 * U-shaped channel with three 90-degree bends:
 *     ____
 *    |    |
 *    |    |  <- Bend 0 (left wall) and Bend 2 (right wall)
 *    |____|  <- Bend 1 (bottom)
 *
 * @return Vector of 3 BendFeature objects
 */
inline std::vector<phase1::BendFeature> createUChannel() {
    std::vector<phase1::BendFeature> bends;

    // Left wall
    phase1::BendFeature left;
    left.id = 0;
    left.angle = 90.0;
    left.length = 100.0;
    left.position = {0.0, 0.0, 0.0};
    left.direction = {1.0, 0.0, 0.0};
    left.normal = {0.0, 0.0, 1.0};
    bends.push_back(left);

    // Bottom
    phase1::BendFeature bottom;
    bottom.id = 1;
    bottom.angle = 90.0;
    bottom.length = 150.0;
    bottom.position = {200.0, 0.0, 0.0};  // 200mm spacing
    bottom.direction = {1.0, 0.0, 0.0};
    bottom.normal = {0.0, 0.0, 1.0};
    bends.push_back(bottom);

    // Right wall
    phase1::BendFeature right;
    right.id = 2;
    right.angle = 90.0;
    right.length = 100.0;
    right.position = {400.0, 0.0, 0.0};  // 200mm from bottom
    right.direction = {1.0, 0.0, 0.0};
    right.normal = {0.0, 0.0, 1.0};
    bends.push_back(right);

    return bends;
}

/**
 * @brief Create box bend configuration
 *
 * Box with four 90-degree bends arranged linearly:
 * Bends are positioned far enough apart to avoid geometric conflicts.
 *
 * @return Vector of 4 BendFeature objects
 */
inline std::vector<phase1::BendFeature> createBox() {
    std::vector<phase1::BendFeature> bends;

    for (int i = 0; i < 4; i++) {
        phase1::BendFeature b;
        b.id = i;
        b.angle = 90.0;
        b.length = 100.0;
        b.position = {i * 150.0, 0.0, 0.0};  // 150mm spacing
        b.direction = {1.0, 0.0, 0.0};
        b.normal = {0.0, 0.0, 1.0};
        bends.push_back(b);
    }

    return bends;
}

/**
 * @brief Create complex panel bend configuration
 *
 * Complex panel with 7 bends at various angles:
 * - Mixed angles: 90, 45, 135, 60 degrees
 * - Varying lengths: 80-200mm
 * - Different orientations
 *
 * Tests solver with non-trivial constraint combinations.
 *
 * @return Vector of 7 BendFeature objects
 */
inline std::vector<phase1::BendFeature> createComplexPanel() {
    std::vector<phase1::BendFeature> bends;

    double angles[] = {90.0, 45.0, 90.0, 135.0, 90.0, 60.0, 90.0};
    double lengths[] = {150.0, 80.0, 120.0, 100.0, 200.0, 90.0, 110.0};

    for (int i = 0; i < 7; i++) {
        phase1::BendFeature b;
        b.id = i;
        b.angle = angles[i];
        b.length = lengths[i];
        b.position = {i * 200.0, 0.0, 0.0};  // 200mm spacing
        b.direction = {1.0, 0.0, 0.0};
        b.normal = {0.0, 0.0, 1.0};
        bends.push_back(b);
    }

    return bends;
}

/**
 * @brief Create simple single-bend for edge case testing
 *
 * @return Vector of 1 BendFeature object
 */
inline std::vector<phase1::BendFeature> createSingleBend() {
    std::vector<phase1::BendFeature> bends;

    phase1::BendFeature b;
    b.id = 0;
    b.angle = 90.0;
    b.length = 100.0;
    b.position = {0.0, 0.0, 0.0};
    b.direction = {1.0, 0.0, 0.0};
    b.normal = {0.0, 0.0, 1.0};
    bends.push_back(b);

    return bends;
}

/**
 * @brief Create Z-profile bend configuration
 *
 * Z-shaped profile with two opposing 90-degree bends:
 *       ____
 *      |
 *   ___|  <- Step
 *
 * @return Vector of 2 BendFeature objects
 */
inline std::vector<phase1::BendFeature> createZProfile() {
    std::vector<phase1::BendFeature> bends;

    phase1::BendFeature b0;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 80.0;
    b0.position = {0.0, 0.0, 0.0};
    b0.direction = {1.0, 0.0, 0.0};
    b0.normal = {0.0, 0.0, 1.0};
    bends.push_back(b0);

    phase1::BendFeature b1;
    b1.id = 1;
    b1.angle = -90.0;  // Opposite direction
    b1.length = 80.0;
    b1.position = {200.0, 0.0, 0.0};  // 200mm spacing
    b1.direction = {1.0, 0.0, 0.0};
    b1.normal = {0.0, 0.0, -1.0};
    bends.push_back(b1);

    return bends;
}

/**
 * @brief Create hat profile bend configuration
 *
 * Hat-shaped profile with four bends:
 *     ______
 *    |      |
 * ___|      |___
 *
 * @return Vector of 4 BendFeature objects
 */
inline std::vector<phase1::BendFeature> createHatProfile() {
    std::vector<phase1::BendFeature> bends;

    for (int i = 0; i < 4; i++) {
        phase1::BendFeature b;
        b.id = i;
        b.angle = 90.0;
        b.length = 60.0 + i * 10.0;  // Varying lengths
        b.position = {i * 150.0, 0.0, 0.0};  // 150mm spacing
        b.direction = {1.0, 0.0, 0.0};
        b.normal = {0.0, 0.0, 1.0};
        bends.push_back(b);
    }

    return bends;
}

} // namespace fixtures
} // namespace phase2
} // namespace openpanelcam
