#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <openpanelcam/unfold/bend_allowance.h>

using namespace openpanelcam;
using Catch::Approx;

TEST_CASE("BendAllowance: 90-degree bend standard formula", "[unfold][ba]") {
    BendAllowanceCalculator calc;

    // R=2mm, T=2mm, angle=90°, K=0.44
    // BA = (π/180) × 90 × (2 + 0.44 × 2) = π/2 × 2.88 = 4.523893...
    auto result = calc.compute(2.0, 90.0, 2.0, 0.44);

    CHECK(result.bendAllowance == Approx(4.524).margin(0.01));
    CHECK(result.bendAllowance > 0.0);
}

TEST_CASE("BendAllowance: bend deduction calculation", "[unfold][ba]") {
    BendAllowanceCalculator calc;

    // R=2mm, T=2mm, angle=90°, K=0.44
    // OSSB = tan(45°) × (2 + 2) = 4.0
    // BD = 2 × OSSB - BA = 8.0 - 4.524 = 3.476
    auto result = calc.compute(2.0, 90.0, 2.0, 0.44);

    CHECK(result.outsideSetback == Approx(4.0).margin(0.01));
    CHECK(result.bendDeduction == Approx(3.476).margin(0.02));
}

TEST_CASE("BendAllowance: 45-degree bend", "[unfold][ba]") {
    BendAllowanceCalculator calc;

    // R=1mm, T=1mm, angle=45°, K=0.42
    // BA = (π/180) × 45 × (1 + 0.42 × 1) = 0.7854 × 1.42 = 1.115
    auto result = calc.compute(1.0, 45.0, 1.0, 0.42);

    CHECK(result.bendAllowance == Approx(1.115).margin(0.01));
}

TEST_CASE("BendAllowance: 180-degree hem", "[unfold][ba]") {
    BendAllowanceCalculator calc;

    // Hem: angle=180°, R≈0, T=1mm
    // Standard formula: BA = π × (0 + 0.33 × 1) = 1.036
    auto result = calc.compute(0.0, 180.0, 1.0, 0.33);

    CHECK(result.bendAllowance == Approx(1.036).margin(0.02));
}

TEST_CASE("BendAllowance: hem empirical override", "[unfold][ba]") {
    BendAllowanceCalculator calc;
    calc.setUseHemEmpirical(true);

    // Closed hem: BD ≈ 0.43 × T
    auto result = calc.computeHem(1.0, 180.0); // T=1mm

    CHECK(result.bendDeduction == Approx(0.43).margin(0.05));
}

TEST_CASE("BendAllowance: zero radius edge case", "[unfold][ba]") {
    BendAllowanceCalculator calc;

    auto result = calc.compute(0.0, 90.0, 1.0, 0.33);

    CHECK(result.bendAllowance > 0.0); // K×T contributes even at R=0
}

TEST_CASE("BendAllowance: with DIN6935 K-factor auto-lookup", "[unfold][ba]") {
    BendAllowanceCalculator calc;
    calc.setUseDIN6935(true);

    // R=2mm, T=2mm → r/s=1.0 → K=0.42 (from DIN 6935)
    // BA = (π/180) × 90 × (2 + 0.42 × 2) = π/2 × 2.84 = 4.461
    auto result = calc.computeWithAutoK(2.0, 90.0, 2.0);

    CHECK(result.kFactor == Approx(0.42).margin(0.01));
    CHECK(result.bendAllowance == Approx(4.461).margin(0.02));
}

TEST_CASE("BendAllowance: flat pattern length for two-leg part", "[unfold][ba]") {
    BendAllowanceCalculator calc;

    // L-bracket: two legs of 50mm each, 90° bend, R=2mm, T=2mm, K=0.44
    // Flat = Leg1 + Leg2 - BD
    auto ba = calc.compute(2.0, 90.0, 2.0, 0.44);
    double flatLength = 50.0 + 50.0 - ba.bendDeduction;

    CHECK(flatLength == Approx(96.524).margin(0.05));
    CHECK(flatLength < 100.0); // Must be shorter than sum of legs
}
