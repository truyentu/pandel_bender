#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <openpanelcam/unfold/k_factor_table.h>

using namespace openpanelcam;
using Catch::Approx;

TEST_CASE("DIN6935: K-factor at exact table entries", "[unfold][kfactor]") {
    DIN6935Table table;

    CHECK(table.getKFactor(0.10) == Approx(0.32));
    CHECK(table.getKFactor(0.50) == Approx(0.38));
    CHECK(table.getKFactor(1.00) == Approx(0.42));
    CHECK(table.getKFactor(2.00) == Approx(0.445));
    CHECK(table.getKFactor(5.00) == Approx(0.48));
    CHECK(table.getKFactor(10.0) == Approx(0.50));
}

TEST_CASE("DIN6935: K-factor interpolation between entries", "[unfold][kfactor]") {
    DIN6935Table table;

    // Midpoint between 1.0 (0.42) and 2.0 (0.445) → ~0.4325
    double k = table.getKFactor(1.5);
    CHECK(k > 0.42);
    CHECK(k < 0.445);
    CHECK(k == Approx(0.4325).margin(0.005));
}

TEST_CASE("DIN6935: K-factor clamped at boundaries", "[unfold][kfactor]") {
    DIN6935Table table;

    // Below minimum r/s
    CHECK(table.getKFactor(0.0) == Approx(0.32));
    CHECK(table.getKFactor(0.05) == Approx(0.32));

    // Above maximum r/s
    CHECK(table.getKFactor(15.0) == Approx(0.50));
    CHECK(table.getKFactor(100.0) == Approx(0.50));
}

TEST_CASE("DIN6935: K-factor from radius and thickness", "[unfold][kfactor]") {
    DIN6935Table table;

    // R=2mm, T=2mm → r/s = 1.0 → K ≈ 0.42
    CHECK(table.getKFactor(2.0, 2.0) == Approx(0.42));

    // R=1mm, T=1mm → r/s = 1.0 → K ≈ 0.42
    CHECK(table.getKFactor(1.0, 1.0) == Approx(0.42));

    // R=5mm, T=1mm → r/s = 5.0 → K ≈ 0.48
    CHECK(table.getKFactor(5.0, 1.0) == Approx(0.48));
}

TEST_CASE("MaterialKFactorTable: material-specific lookups", "[unfold][kfactor]") {
    MaterialKFactorTable table;

    // Default materials at r/t=1.0
    CHECK(table.getKFactor("mild_steel", 1.0) == Approx(0.38).margin(0.02));
    CHECK(table.getKFactor("stainless_steel", 1.0) == Approx(0.40).margin(0.02));
    CHECK(table.getKFactor("aluminum", 1.0) == Approx(0.38).margin(0.02));

    // Higher r/t ratios
    CHECK(table.getKFactor("mild_steel", 3.0) > 0.40);
    CHECK(table.getKFactor("stainless_steel", 3.0) > 0.42);
}

TEST_CASE("MaterialKFactorTable: unknown material falls back to DIN6935", "[unfold][kfactor]") {
    MaterialKFactorTable table;

    double kUnknown = table.getKFactor("unobtainium", 1.0);
    DIN6935Table din;
    double kDIN = din.getKFactor(1.0);

    CHECK(kUnknown == Approx(kDIN));
}
