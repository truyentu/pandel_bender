#include <openpanelcam/unfold/bend_allowance.h>
#include <cmath>

namespace openpanelcam {

static constexpr double PI = 3.14159265358979323846;

BendAllowanceCalculator::BendAllowanceCalculator() = default;

BendAllowanceResult BendAllowanceCalculator::compute(
    double radius, double angleDeg, double thickness, double kFactor) const
{
    BendAllowanceResult result;
    result.kFactor = kFactor;
    result.neutralRadius = radius + kFactor * thickness;

    // BA = (π/180) × angle × (R + K × T)
    result.bendAllowance = (PI / 180.0) * angleDeg * result.neutralRadius;

    // OSSB = tan(angle/2) × (R + T)
    double halfAngleRad = (angleDeg / 2.0) * PI / 180.0;
    result.outsideSetback = std::tan(halfAngleRad) * (radius + thickness);

    // BD = 2 × OSSB - BA
    result.bendDeduction = 2.0 * result.outsideSetback - result.bendAllowance;

    return result;
}

BendAllowanceResult BendAllowanceCalculator::computeWithAutoK(
    double radius, double angleDeg, double thickness) const
{
    double kFactor = m_din6935.getKFactor(radius, thickness);
    return compute(radius, angleDeg, thickness, kFactor);
}

BendAllowanceResult BendAllowanceCalculator::computeWithMaterial(
    double radius, double angleDeg, double thickness,
    const std::string& material) const
{
    double rt = (thickness > 0.0) ? radius / thickness : 1.0;
    double kFactor = m_materialTable.getKFactor(material, rt);
    return compute(radius, angleDeg, thickness, kFactor);
}

BendAllowanceResult BendAllowanceCalculator::computeHem(
    double thickness, double angleDeg) const
{
    if (m_useHemEmpirical && angleDeg >= 170.0) {
        // Empirical: closed hem BD ≈ 0.43 × T
        BendAllowanceResult result;
        result.kFactor = 0.33;
        double radius = 0.0; // closed hem
        result.neutralRadius = result.kFactor * thickness;
        result.bendAllowance = (PI / 180.0) * angleDeg * result.neutralRadius;
        double halfAngleRad = (angleDeg / 2.0) * PI / 180.0;
        result.outsideSetback = std::tan(halfAngleRad) * thickness;
        result.bendDeduction = 0.43 * thickness; // empirical override
        return result;
    }
    return compute(0.0, angleDeg, thickness, 0.33);
}

void BendAllowanceCalculator::setUseDIN6935(bool use) { m_useDIN6935 = use; }
void BendAllowanceCalculator::setUseHemEmpirical(bool use) { m_useHemEmpirical = use; }

} // namespace openpanelcam
