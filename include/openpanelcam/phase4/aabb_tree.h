#pragma once

#include "types.h"
#include <vector>
#include <memory>
#include <functional>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief BVH node for AABB tree hierarchy
 *
 * Paper structure (simplified for our model):
 * Root (Part AABB) -> Flange AABBs (leaves)
 *
 * Binary tree: internal nodes hold merged AABBs,
 * leaves hold individual flange AABBs with bendId.
 */
struct AABBTreeNode {
    AABB bounds;
    int bendId = -1;           // -1 for internal nodes, >= 0 for leaves
    int left = -1;             // Index of left child (-1 if leaf)
    int right = -1;            // Index of right child (-1 if leaf)

    bool isLeaf() const { return left == -1 && right == -1; }
};

/**
 * @brief AABB Tree for O(n log n) broad-phase collision detection
 *
 * From paper: BVH hierarchy for sheet metal parts.
 * Uses incremental insertion with Surface Area Heuristic (SAH)
 * for choosing optimal split.
 */
class AABBTree {
public:
    AABBTree() = default;

    /**
     * @brief Insert a new AABB (bent flange) into the tree
     * @param bounds The AABB of the flange
     * @param bendId The bend identifier
     */
    void insert(const AABB& bounds, int bendId);

    /**
     * @brief Remove a flange from the tree by bendId
     * @param bendId The bend to remove
     * @return true if found and removed
     */
    bool remove(int bendId);

    /**
     * @brief Query all leaves that overlap with the given AABB
     * @param query The AABB to test against
     * @param results Output: bendIds of overlapping flanges
     */
    void query(const AABB& query, std::vector<int>& results) const;

    /**
     * @brief Get the root bounding box (entire part)
     */
    AABB getRootBounds() const;

    /**
     * @brief Get number of leaves (flanges) in the tree
     */
    int size() const { return m_leafCount; }

    /**
     * @brief Clear the tree
     */
    void clear();

private:
    std::vector<AABBTreeNode> m_nodes;
    int m_root = -1;
    int m_leafCount = 0;

    int allocateNode();
    void queryRecursive(int nodeIdx, const AABB& query,
                        std::vector<int>& results) const;
    int insertLeaf(int leafIdx);
    void removeLeaf(int leafIdx);
    int findBestSibling(int leafIdx) const;
    void refitAncestors(int nodeIdx);

    // SAH cost: surface area of merged AABB
    static double surfaceArea(const AABB& box);
};

} // namespace phase4
} // namespace openpanelcam
