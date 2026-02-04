#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/types.h"

using namespace openpanelcam::phase2;

TEST_CASE("ConstraintType enum values exist", "[phase2][types]") {
    ConstraintType ct1 = ConstraintType::GEOMETRIC;
    ConstraintType ct2 = ConstraintType::BOX_CLOSING;
    ConstraintType ct3 = ConstraintType::SEQUENTIAL;
    REQUIRE(true);
}

TEST_CASE("DeadZoneType enum values exist", "[phase2][types]") {
    DeadZoneType dz1 = DeadZoneType::STANDING_FLANGE;
    DeadZoneType dz2 = DeadZoneType::SAFETY_MARGIN;
    DeadZoneType dz3 = DeadZoneType::ABA_INTERFERENCE;
    REQUIRE(true);
}
