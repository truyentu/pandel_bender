#pragma once

#include "openpanelcam/phase2/phase1_mock.h"
#include <vector>

namespace openpanelcam {
namespace phase2 {

/**
 * @brief Adapter class for integration with Phase 1 output
 *
 * This adapter provides a bridge between Phase 1 (FAG/bend feature extraction)
 * and Phase 2 (constraint solving). When Phase 1 is complete, the convert()
 * method will transform FAG data into BendFeature structures for Phase 2.
 *
 * Currently provides mock bend generation for testing Phase 2 in isolation.
 *
 * Usage:
 *   // For testing with mock data:
 *   auto bends = Phase1Adapter::createMockBends(5);
 *   ConstraintSolver solver;
 *   auto output = solver.solve(bends);
 *
 *   // Future integration with Phase 1:
 *   // auto bends = Phase1Adapter::convert(fag);
 */
class Phase1Adapter {
public:
    // TODO: Implement when Phase 1 is complete
    // static std::vector<phase1::BendFeature> convert(const FAG& fag);

    /**
     * @brief Create mock bend features for testing
     *
     * Generates a set of bend features with predictable properties
     * for unit testing and integration testing of Phase 2.
     *
     * @param count Number of mock bends to generate
     * @return Vector of mock bend features
     *
     * Generated bends have:
     * - Sequential IDs from 0 to count-1
     * - 90 degree bend angles
     * - Lengths from 100mm, increasing by 20mm per bend
     * - Positions spaced 150mm apart along X-axis
     * - Alternating Y positions (0 or 100mm)
     * - Alternating bend directions (X or Y axis)
     */
    static std::vector<phase1::BendFeature> createMockBends(int count) {
        std::vector<phase1::BendFeature> bends;
        bends.reserve(static_cast<size_t>(count));

        for (int i = 0; i < count; i++) {
            phase1::BendFeature bend;
            bend.id = i;
            bend.angle = 90.0;
            bend.length = 100.0 + i * 20.0;
            bend.position.x = i * 150.0;
            bend.position.y = (i % 2) * 100.0;
            bend.position.z = 0.0;
            bend.direction.x = (i % 2 == 0) ? 1.0 : 0.0;
            bend.direction.y = (i % 2 == 1) ? 1.0 : 0.0;
            bend.direction.z = 0.0;
            bends.push_back(bend);
        }

        return bends;
    }

    /**
     * @brief Create mock bends with custom angles
     *
     * @param count Number of bends to generate
     * @param angles Vector of angles for each bend (cycled if shorter than count)
     * @return Vector of mock bend features with specified angles
     */
    static std::vector<phase1::BendFeature> createMockBendsWithAngles(
        int count,
        const std::vector<double>& angles
    ) {
        std::vector<phase1::BendFeature> bends;
        bends.reserve(static_cast<size_t>(count));

        for (int i = 0; i < count; i++) {
            phase1::BendFeature bend;
            bend.id = i;
            bend.angle = angles.empty() ? 90.0 : angles[i % angles.size()];
            bend.length = 100.0 + i * 20.0;
            bend.position.x = i * 150.0;
            bend.position.y = (i % 2) * 100.0;
            bend.position.z = 0.0;
            bend.direction.x = (i % 2 == 0) ? 1.0 : 0.0;
            bend.direction.y = (i % 2 == 1) ? 1.0 : 0.0;
            bend.direction.z = 0.0;
            bends.push_back(bend);
        }

        return bends;
    }

    /**
     * @brief Create a single mock bend with custom properties
     *
     * @param id Bend identifier
     * @param angle Bend angle in degrees
     * @param length Bend line length in mm
     * @param posX X position in mm
     * @param posY Y position in mm
     * @param posZ Z position in mm
     * @return Single bend feature
     */
    static phase1::BendFeature createSingleBend(
        int id,
        double angle,
        double length,
        double posX = 0.0,
        double posY = 0.0,
        double posZ = 0.0
    ) {
        phase1::BendFeature bend;
        bend.id = id;
        bend.angle = angle;
        bend.length = length;
        bend.position.x = posX;
        bend.position.y = posY;
        bend.position.z = posZ;
        bend.direction.x = 1.0;
        bend.direction.y = 0.0;
        bend.direction.z = 0.0;
        return bend;
    }

    /**
     * @brief Create a box-closing scenario for testing
     *
     * Generates bends that would form a closed box, which creates
     * challenging constraints for the solver (restricted access from
     * multiple directions).
     *
     * @return Vector of 4 bends forming a box pattern
     */
    static std::vector<phase1::BendFeature> createBoxClosingScenario() {
        std::vector<phase1::BendFeature> bends;
        bends.reserve(4);

        // Four 90-degree bends forming a box
        // Positioned at corners of a 200mm x 200mm square
        double size = 200.0;

        for (int i = 0; i < 4; i++) {
            phase1::BendFeature bend;
            bend.id = i;
            bend.angle = 90.0;
            bend.length = size;

            // Position at corners
            switch (i) {
                case 0:
                    bend.position.x = 0.0;
                    bend.position.y = 0.0;
                    bend.direction.x = 1.0;
                    bend.direction.y = 0.0;
                    break;
                case 1:
                    bend.position.x = size;
                    bend.position.y = 0.0;
                    bend.direction.x = 0.0;
                    bend.direction.y = 1.0;
                    break;
                case 2:
                    bend.position.x = size;
                    bend.position.y = size;
                    bend.direction.x = -1.0;
                    bend.direction.y = 0.0;
                    break;
                case 3:
                    bend.position.x = 0.0;
                    bend.position.y = size;
                    bend.direction.x = 0.0;
                    bend.direction.y = -1.0;
                    break;
            }
            bend.position.z = 0.0;
            bend.direction.z = 0.0;

            bends.push_back(bend);
        }

        return bends;
    }
};

} // namespace phase2
} // namespace openpanelcam
