#include "openpanelcam/phase4/aabb_tree.h"
#include <algorithm>
#include <limits>
#include <stack>

namespace openpanelcam {
namespace phase4 {

int AABBTree::allocateNode() {
    m_nodes.push_back(AABBTreeNode{});
    return static_cast<int>(m_nodes.size()) - 1;
}

double AABBTree::surfaceArea(const AABB& box) {
    double dx = box.maxX - box.minX;
    double dy = box.maxY - box.minY;
    double dz = box.maxZ - box.minZ;
    if (dx < 0 || dy < 0 || dz < 0) return 0.0;
    return 2.0 * (dx * dy + dy * dz + dz * dx);
}

void AABBTree::insert(const AABB& bounds, int bendId) {
    int leafIdx = allocateNode();
    m_nodes[leafIdx].bounds = bounds;
    m_nodes[leafIdx].bendId = bendId;
    m_leafCount++;

    if (m_root == -1) {
        m_root = leafIdx;
        return;
    }

    // Find best sibling using SAH
    int bestSibling = findBestSibling(leafIdx);

    // Create new internal node
    int newParent = allocateNode();
    m_nodes[newParent].bounds = AABB::merge(m_nodes[bestSibling].bounds, bounds);
    m_nodes[newParent].left = bestSibling;
    m_nodes[newParent].right = leafIdx;

    // If sibling was root, new parent becomes root
    if (bestSibling == m_root) {
        m_root = newParent;
    } else {
        // Find and update the old parent of the sibling
        // Simple approach: rebuild parent linkage
        // For our use case (< 32 bends), linear search is fine
        for (int i = 0; i < static_cast<int>(m_nodes.size()); i++) {
            if (i == newParent || i == leafIdx) continue;
            if (m_nodes[i].left == bestSibling) {
                m_nodes[i].left = newParent;
                // Refit this ancestor
                m_nodes[i].bounds = AABB::merge(
                    m_nodes[m_nodes[i].left].bounds,
                    m_nodes[m_nodes[i].right].bounds);
                break;
            }
            if (m_nodes[i].right == bestSibling) {
                m_nodes[i].right = newParent;
                m_nodes[i].bounds = AABB::merge(
                    m_nodes[m_nodes[i].left].bounds,
                    m_nodes[m_nodes[i].right].bounds);
                break;
            }
        }
    }
}

int AABBTree::findBestSibling(int leafIdx) const {
    if (m_root == -1) return -1;

    const AABB& leafBounds = m_nodes[leafIdx].bounds;
    int bestIdx = m_root;
    double bestCost = surfaceArea(AABB::merge(m_nodes[m_root].bounds, leafBounds));

    // For small trees, just check all leaves
    for (int i = 0; i < static_cast<int>(m_nodes.size()); i++) {
        if (i == leafIdx) continue;
        if (!m_nodes[i].isLeaf() && m_nodes[i].bendId == -1 &&
            m_nodes[i].left == -1 && m_nodes[i].right == -1) continue;  // Skip unused

        double cost = surfaceArea(AABB::merge(m_nodes[i].bounds, leafBounds));
        if (cost < bestCost) {
            bestCost = cost;
            bestIdx = i;
        }
    }

    return bestIdx;
}

bool AABBTree::remove(int bendId) {
    // Find the leaf with this bendId
    int leafIdx = -1;
    for (int i = 0; i < static_cast<int>(m_nodes.size()); i++) {
        if (m_nodes[i].bendId == bendId) {
            leafIdx = i;
            break;
        }
    }
    if (leafIdx == -1) return false;

    m_leafCount--;

    // If it's the only node (root)
    if (leafIdx == m_root) {
        m_root = -1;
        return true;
    }

    // Find parent of this leaf
    for (int i = 0; i < static_cast<int>(m_nodes.size()); i++) {
        if (m_nodes[i].left == leafIdx) {
            int sibling = m_nodes[i].right;
            // Replace parent with sibling
            if (i == m_root) {
                m_root = sibling;
            } else {
                // Find grandparent
                for (int j = 0; j < static_cast<int>(m_nodes.size()); j++) {
                    if (m_nodes[j].left == i) {
                        m_nodes[j].left = sibling;
                        m_nodes[j].bounds = AABB::merge(
                            m_nodes[m_nodes[j].left].bounds,
                            m_nodes[m_nodes[j].right].bounds);
                        break;
                    }
                    if (m_nodes[j].right == i) {
                        m_nodes[j].right = sibling;
                        m_nodes[j].bounds = AABB::merge(
                            m_nodes[m_nodes[j].left].bounds,
                            m_nodes[m_nodes[j].right].bounds);
                        break;
                    }
                }
            }
            // Mark removed nodes as unused
            m_nodes[leafIdx].bendId = -1;
            m_nodes[leafIdx].left = -1;
            m_nodes[leafIdx].right = -1;
            m_nodes[i].bendId = -1;
            m_nodes[i].left = -1;
            m_nodes[i].right = -1;
            return true;
        }
        if (m_nodes[i].right == leafIdx) {
            int sibling = m_nodes[i].left;
            if (i == m_root) {
                m_root = sibling;
            } else {
                for (int j = 0; j < static_cast<int>(m_nodes.size()); j++) {
                    if (m_nodes[j].left == i) {
                        m_nodes[j].left = sibling;
                        m_nodes[j].bounds = AABB::merge(
                            m_nodes[m_nodes[j].left].bounds,
                            m_nodes[m_nodes[j].right].bounds);
                        break;
                    }
                    if (m_nodes[j].right == i) {
                        m_nodes[j].right = sibling;
                        m_nodes[j].bounds = AABB::merge(
                            m_nodes[m_nodes[j].left].bounds,
                            m_nodes[m_nodes[j].right].bounds);
                        break;
                    }
                }
            }
            m_nodes[leafIdx].bendId = -1;
            m_nodes[leafIdx].left = -1;
            m_nodes[leafIdx].right = -1;
            m_nodes[i].bendId = -1;
            m_nodes[i].left = -1;
            m_nodes[i].right = -1;
            return true;
        }
    }

    return false;
}

void AABBTree::query(const AABB& queryBox, std::vector<int>& results) const {
    results.clear();
    if (m_root == -1) return;
    queryRecursive(m_root, queryBox, results);
}

void AABBTree::queryRecursive(int nodeIdx, const AABB& queryBox,
                               std::vector<int>& results) const
{
    if (nodeIdx < 0 || nodeIdx >= static_cast<int>(m_nodes.size())) return;

    const auto& node = m_nodes[nodeIdx];

    if (!node.bounds.overlaps(queryBox)) {
        return;  // Prune: no overlap with this subtree
    }

    if (node.isLeaf()) {
        if (node.bendId >= 0) {
            results.push_back(node.bendId);
        }
        return;
    }

    if (node.left >= 0) queryRecursive(node.left, queryBox, results);
    if (node.right >= 0) queryRecursive(node.right, queryBox, results);
}

AABB AABBTree::getRootBounds() const {
    if (m_root == -1) return AABB();
    return m_nodes[m_root].bounds;
}

void AABBTree::clear() {
    m_nodes.clear();
    m_root = -1;
    m_leafCount = 0;
}

} // namespace phase4
} // namespace openpanelcam
