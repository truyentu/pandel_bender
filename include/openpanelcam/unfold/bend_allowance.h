#pragma once

#include <openpanelcam/unfold/k_factor_table.h>

namespace openpanelcam {

struct BendAllowanceResult {
    double bendAllowance = 0.0;   // BA: arc length at neutral axis (mm)
    double outsideSetback = 0.0;  // OSSB: tangent point to mold line (mm)
    double bendDeduction = 0.0;   // BD: amount to subtract from outside dims (mm)
    double kFactor = 0.0;         // K-factor used
    double neutralRadius = 0.0;   // R + K × T (mm)
};

class BendAllowanceCalculator {
public:
    BendAllowanceCalculator();

    // Core computation with explicit K-factor
    BendAllowanceResult compute(double radius, double angleDeg,
                                double thickness, double kFactor) const;

    // Auto-lookup K-factor from DIN 6935 table
    BendAllowanceResult computeWithAutoK(double radius, double angleDeg,
                                         double thickness) const;

    // Auto-lookup K-factor from material table
    BendAllowanceResult computeWithMaterial(double radius, double angleDeg,
                                            double thickness,
                                            const std::string& material) const;

    // Hem-specific (empirical override)
    BendAllowanceResult computeHem(double thickness, double angleDeg = 180.0) const;

    // Configuration
    void setUseDIN6935(bool use);
    void setUseHemEmpirical(bool use);

private:
    DIN6935Table m_din6935;
    MaterialKFactorTable m_materialTable;
    bool m_useDIN6935 = true;
    bool m_useHemEmpirical = true;
};

} // namespace openpanelcam
