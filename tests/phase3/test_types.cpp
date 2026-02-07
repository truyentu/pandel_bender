#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase3/types.h"

using namespace openpanelcam::phase3;

TEST_CASE("Orientation enum values", "[phase3][types]") {
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_0) == 0);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_90) == 1);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_180) == 2);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_270) == 3);
}

TEST_CASE("orientationToDegrees converts correctly", "[phase3][types]") {
    REQUIRE(orientationToDegrees(Orientation::DEG_0) == 0.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_90) == 90.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_180) == 180.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_270) == 270.0);
}

TEST_CASE("rotateClockwise cycles through orientations", "[phase3][types]") {
    REQUIRE(rotateClockwise(Orientation::DEG_0) == Orientation::DEG_90);
    REQUIRE(rotateClockwise(Orientation::DEG_90) == Orientation::DEG_180);
    REQUIRE(rotateClockwise(Orientation::DEG_180) == Orientation::DEG_270);
    REQUIRE(rotateClockwise(Orientation::DEG_270) == Orientation::DEG_0);
}

TEST_CASE("rotateOpposite returns 180 degree rotation", "[phase3][types]") {
    REQUIRE(rotateOpposite(Orientation::DEG_0) == Orientation::DEG_180);
    REQUIRE(rotateOpposite(Orientation::DEG_90) == Orientation::DEG_270);
    REQUIRE(rotateOpposite(Orientation::DEG_180) == Orientation::DEG_0);
    REQUIRE(rotateOpposite(Orientation::DEG_270) == Orientation::DEG_90);
}

TEST_CASE("ActionType enum exists", "[phase3][types]") {
    ActionType bend = ActionType::BEND;
    ActionType rotate = ActionType::ROTATE;
    ActionType aba = ActionType::ABA_RECONFIG;
    ActionType repo = ActionType::REPOSITION;

    REQUIRE(bend != rotate);
    REQUIRE(rotate != aba);
    REQUIRE(aba != repo);
}

TEST_CASE("RepoReason enum exists", "[phase3][types]") {
    REQUIRE(RepoReason::NONE != RepoReason::GRIP_AREA_EXHAUSTED);
    REQUIRE(RepoReason::BOX_CLOSING != RepoReason::COM_OUTSIDE_GRIP);
}
