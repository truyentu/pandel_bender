#pragma once

#include <openpanelcam/unfold/types.h>
#include <openpanelcam/unfold/bend_allowance.h>
#include <openpanelcam/unfold/unfold_tree.h>
#include <openpanelcam/unfold/k_factor_table.h>

namespace openpanelcam {

class FaceAdjacencyGraph;

class SheetMetalUnfolder {
public:
    explicit SheetMetalUnfolder(const UnfoldConfig& config);

    // Main unfold pipeline
    UnfoldResult unfold(const FaceAdjacencyGraph& fag, int baseFaceId);

private:
    void computeBendZones(const UnfoldTree& tree,
                          const FaceAdjacencyGraph& fag,
                          UnfoldResult& result);

    void computeTransforms(UnfoldTree& tree,
                           const FaceAdjacencyGraph& fag,
                           const std::vector<BendZoneInfo>& bendZones);

    double resolveKFactor(double radius) const;

    UnfoldConfig m_config;
    BendAllowanceCalculator m_baCalc;
    DIN6935Table m_din6935;
    MaterialKFactorTable m_materialTable;
    UnfoldTreeBuilder m_treeBuilder;
};

} // namespace openpanelcam
