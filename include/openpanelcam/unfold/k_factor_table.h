#pragma once

#include <string>
#include <vector>
#include <utility>

namespace openpanelcam {

class DIN6935Table {
public:
    DIN6935Table();

    // Get K-factor from r/s ratio (interpolated)
    double getKFactor(double rs_ratio) const;

    // Convenience: get K-factor from radius and thickness
    double getKFactor(double radius, double thickness) const;

private:
    // Table entries: {r/s ratio, K-factor}
    std::vector<std::pair<double, double>> m_table;
};

class MaterialKFactorTable {
public:
    MaterialKFactorTable();

    // Get K-factor for material at given r/t ratio
    double getKFactor(const std::string& material, double rt_ratio) const;

    // Add custom material entry
    void addMaterial(const std::string& name,
                     const std::vector<std::pair<double, double>>& rtToK);

    bool hasMaterial(const std::string& name) const;

private:
    struct MaterialEntry {
        std::string name;
        std::vector<std::pair<double, double>> table; // {r/t, K}
    };

    std::vector<MaterialEntry> m_materials;
    DIN6935Table m_din6935; // fallback
};

} // namespace openpanelcam
