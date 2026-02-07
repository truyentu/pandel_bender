#include "openpanelcam/phase3/astar_search.h"
#include <algorithm>

namespace openpanelcam {
namespace phase3 {

AStarSearch::AStarSearch(const phase2::Phase2Output& phase2Output,
                         const std::vector<phase1::BendFeature>& bends)
    : m_phase2(phase2Output)
    , m_bends(bends)
    , m_zobrist(12345)
{
    m_successor = std::make_unique<SuccessorGenerator>(
        m_phase2.precedenceGraph,
        m_bends,
        m_phase2.graspConstraints
    );
}

bool AStarSearch::isGoal(const SearchState& state) const {
    return state.isGoal(static_cast<int>(m_bends.size()));
}

Phase3Output AStarSearch::search(const SearchConfig& config) {
    Phase3Output output;
    m_stats = SequencerStatistics();
    auto startTime = std::chrono::high_resolution_clock::now();

    if (m_bends.empty()) {
        output.success = true;
        output.optimal = true;
        auto endTime = std::chrono::high_resolution_clock::now();
        m_stats.searchTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        output.stats = m_stats;
        return output;
    }

    // All nodes stored here for path reconstruction
    std::vector<SearchNode> allNodes;

    // Priority queue: (f-score, node index) - min heap
    auto cmp = [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        decltype(cmp)> openQueue(cmp);

    // Zobrist hash -> best g-cost seen for this state
    std::unordered_map<uint64_t, std::pair<int, double>> visited; // hash -> (nodeIdx, bestG)

    // Initialize
    SearchNode startNode = SearchNode::createInitial();
    startNode.h = m_heuristic.estimate(startNode.state, m_bends);

    allNodes.push_back(startNode);
    uint64_t startHash = m_zobrist.hash(startNode.state);
    visited[startHash] = {0, startNode.g};
    openQueue.push({startNode.f(), 0});

    m_stats.nodesGenerated = 1;
    m_stats.maxOpenSetSize = 1;

    while (!openQueue.empty()) {
        // Check limits
        if (m_stats.nodesExpanded >= config.maxNodes) {
            output.errorMessage = "Max nodes reached";
            break;
        }

        auto elapsed = std::chrono::high_resolution_clock::now() - startTime;
        if (std::chrono::duration<double>(elapsed).count() > config.timeoutSeconds) {
            output.errorMessage = "Timeout";
            break;
        }

        auto [fScore, nodeIdx] = openQueue.top();
        openQueue.pop();

        const SearchNode& current = allNodes[nodeIdx];

        // Skip if we've found a better path to this state already
        uint64_t currentHash = m_zobrist.hash(current.state);
        auto it = visited.find(currentHash);
        if (it != visited.end() && current.g > it->second.second) {
            m_stats.duplicatesSkipped++;
            continue;
        }

        m_stats.nodesExpanded++;

        // Goal check
        if (isGoal(current.state)) {
            output = reconstructPath(allNodes, nodeIdx);
            output.success = true;
            output.optimal = true;
            m_stats.solutionDepth = static_cast<int>(output.bendSequence.size());
            break;
        }

        // Generate successors
        auto successors = m_successor->generate(current, nodeIdx, m_costFn, m_heuristic);
        m_stats.nodesGenerated += static_cast<int>(successors.size());

        for (auto& succ : successors) {
            uint64_t hash = m_zobrist.hash(succ.state);

            auto vit = visited.find(hash);
            if (vit != visited.end()) {
                if (succ.g >= vit->second.second) {
                    m_stats.duplicatesSkipped++;
                    continue;
                }
                // Better path found - update
                vit->second = {static_cast<int>(allNodes.size()), succ.g};
            } else {
                visited[hash] = {static_cast<int>(allNodes.size()), succ.g};
            }

            int newIdx = static_cast<int>(allNodes.size());
            allNodes.push_back(succ);
            openQueue.push({succ.f(), newIdx});
        }

        m_stats.maxOpenSetSize = std::max(m_stats.maxOpenSetSize,
                                           static_cast<int>(openQueue.size()));
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    m_stats.searchTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    output.stats = m_stats;

    return output;
}

Phase3Output AStarSearch::reconstructPath(const std::vector<SearchNode>& allNodes,
                                           int goalNodeIndex) {
    Phase3Output output;

    // Trace back from goal to start
    std::vector<int> reverseBends;
    int currentIdx = goalNodeIndex;

    while (currentIdx >= 0) {
        const SearchNode& node = allNodes[currentIdx];
        if (node.lastBendId >= 0) {
            reverseBends.push_back(node.lastBendId);
        }
        currentIdx = node.parentId;
    }

    std::reverse(reverseBends.begin(), reverseBends.end());
    output.bendSequence = reverseBends;

    output.totalCycleTime = allNodes[goalNodeIndex].g;

    // Generate detailed actions
    for (int bendId : output.bendSequence) {
        SequenceAction action;
        action.type = ActionType::BEND;
        action.bendId = bendId;
        for (const auto& bend : m_bends) {
            if (bend.id == bendId) {
                action.duration = m_costFn.bendTime(bend);
                action.description = "Bend " + std::to_string(bendId);
                break;
            }
        }
        output.actions.push_back(action);
    }

    output.stats.solutionDepth = static_cast<int>(output.bendSequence.size());

    return output;
}

} // namespace phase3
} // namespace openpanelcam
