#include <openpanelcam/unfold/k_factor_table.h>
#include <algorithm>
#include <cmath>

namespace openpanelcam {

// ============================================================================
// DIN6935Table
// ============================================================================

DIN6935Table::DIN6935Table() {
    m_table = {
        {0.10, 0.32},
        {0.25, 0.35},
        {0.50, 0.38},
        {1.00, 0.42},
        {2.00, 0.445},
        {3.00, 0.47},
        {4.00, 0.475},
        {5.00, 0.48},
        {10.0, 0.50}
    };
}

double DIN6935Table::getKFactor(double rs_ratio) const {
    if (rs_ratio <= m_table.front().first) return m_table.front().second;
    if (rs_ratio >= m_table.back().first)  return m_table.back().second;

    // Find bracketing entries and interpolate
    for (size_t i = 0; i + 1 < m_table.size(); ++i) {
        if (rs_ratio >= m_table[i].first && rs_ratio <= m_table[i + 1].first) {
            double t = (rs_ratio - m_table[i].first) /
                       (m_table[i + 1].first - m_table[i].first);
            return m_table[i].second + t * (m_table[i + 1].second - m_table[i].second);
        }
    }
    return 0.44; // fallback
}

double DIN6935Table::getKFactor(double radius, double thickness) const {
    if (thickness <= 0.0) return 0.44;
    return getKFactor(radius / thickness);
}

// ============================================================================
// MaterialKFactorTable
// ============================================================================

MaterialKFactorTable::MaterialKFactorTable() {
    // Built-in materials: {r/t ratio, K-factor}
    addMaterial("mild_steel", {
        {0.25, 0.33}, {1.0, 0.38}, {3.0, 0.44}, {5.0, 0.46}, {10.0, 0.50}
    });
    addMaterial("stainless_steel", {
        {0.25, 0.33}, {1.0, 0.40}, {3.0, 0.45}, {5.0, 0.48}, {10.0, 0.50}
    });
    addMaterial("aluminum", {
        {0.25, 0.33}, {1.0, 0.38}, {3.0, 0.40}, {5.0, 0.42}, {10.0, 0.50}
    });
    addMaterial("copper", {
        {0.25, 0.35}, {1.0, 0.41}, {3.0, 0.45}, {5.0, 0.47}, {10.0, 0.50}
    });
}

double MaterialKFactorTable::getKFactor(const std::string& material, double rt_ratio) const {
    for (const auto& entry : m_materials) {
        if (entry.name == material) {
            // Interpolate within material table
            const auto& table = entry.table;
            if (rt_ratio <= table.front().first) return table.front().second;
            if (rt_ratio >= table.back().first)  return table.back().second;

            for (size_t i = 0; i + 1 < table.size(); ++i) {
                if (rt_ratio >= table[i].first && rt_ratio <= table[i + 1].first) {
                    double t = (rt_ratio - table[i].first) /
                               (table[i + 1].first - table[i].first);
                    return table[i].second + t * (table[i + 1].second - table[i].second);
                }
            }
        }
    }
    // Fallback to DIN 6935
    return m_din6935.getKFactor(rt_ratio);
}

void MaterialKFactorTable::addMaterial(
    const std::string& name,
    const std::vector<std::pair<double, double>>& rtToK)
{
    // Replace if exists
    for (auto& entry : m_materials) {
        if (entry.name == name) {
            entry.table = rtToK;
            return;
        }
    }
    m_materials.push_back({name, rtToK});
}

bool MaterialKFactorTable::hasMaterial(const std::string& name) const {
    for (const auto& entry : m_materials) {
        if (entry.name == name) return true;
    }
    return false;
}

} // namespace openpanelcam
