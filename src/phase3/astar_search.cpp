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

    // Beam search fallback if A* didn't find a solution
    if (!output.success && config.useBeamSearch) {
        output.warnings.push_back("A* exhausted (" + output.errorMessage + "), falling back to beam search");
        output = beamSearch(config, startTime);
    }

    return output;
}

Phase3Output AStarSearch::beamSearch(const SearchConfig& config,
                                      std::chrono::high_resolution_clock::time_point startTime) {
    Phase3Output output;
    SequencerStatistics beamStats;
    beamStats.nodesGenerated = 1;

    std::vector<SearchNode> allNodes;

    // Start with initial node
    SearchNode startNode = SearchNode::createInitial();
    startNode.h = m_heuristic.estimate(startNode.state, m_bends);
    allNodes.push_back(startNode);

    // Current beam: indices into allNodes
    std::vector<int> currentBeam = {0};

    int totalBends = static_cast<int>(m_bends.size());

    for (int depth = 0; depth < totalBends; depth++) {
        // Check timeout
        auto elapsed = std::chrono::high_resolution_clock::now() - startTime;
        if (std::chrono::duration<double>(elapsed).count() > config.timeoutSeconds * 2.0) {
            output.errorMessage = "Beam search timeout";
            break;
        }

        // Expand all nodes in current beam
        std::vector<std::pair<double, int>> candidates; // (f-score, nodeIdx)

        for (int beamIdx : currentBeam) {
            const SearchNode& current = allNodes[beamIdx];
            beamStats.nodesExpanded++;

            if (isGoal(current.state)) {
                output = reconstructPath(allNodes, beamIdx);
                output.success = true;
                output.optimal = false; // Beam search is not optimal
                output.warnings.push_back("Solution found by beam search (may not be optimal)");
                auto endTime = std::chrono::high_resolution_clock::now();
                beamStats.searchTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
                beamStats.solutionDepth = static_cast<int>(output.bendSequence.size());
                output.stats = beamStats;
                return output;
            }

            auto successors = m_successor->generate(current, beamIdx, m_costFn, m_heuristic);
            beamStats.nodesGenerated += static_cast<int>(successors.size());

            for (auto& succ : successors) {
                int newIdx = static_cast<int>(allNodes.size());
                allNodes.push_back(succ);
                candidates.push_back({succ.f(), newIdx});
            }
        }

        if (candidates.empty()) break;

        // Sort by f-score and keep only top beamWidth
        std::sort(candidates.begin(), candidates.end());
        int keepCount = std::min(config.beamWidth, static_cast<int>(candidates.size()));

        currentBeam.clear();
        for (int i = 0; i < keepCount; i++) {
            currentBeam.push_back(candidates[i].second);
        }

        beamStats.nodesPruned += static_cast<int>(candidates.size()) - keepCount;
        beamStats.maxOpenSetSize = std::max(beamStats.maxOpenSetSize, keepCount);
    }

    // Check if any node in final beam is a goal
    for (int beamIdx : currentBeam) {
        if (isGoal(allNodes[beamIdx].state)) {
            output = reconstructPath(allNodes, beamIdx);
            output.success = true;
            output.optimal = false;
            output.warnings.push_back("Solution found by beam search (may not be optimal)");
            break;
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    beamStats.searchTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    if (output.success) {
        beamStats.solutionDepth = static_cast<int>(output.bendSequence.size());
    }
    output.stats = beamStats;

    if (!output.success) {
        output.errorMessage = "Beam search failed to find solution";
    }

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

    // Generate detailed actions and track repos
    // Collect nodes in forward order for repo detection
    std::vector<int> forwardNodeIndices;
    {
        int idx = goalNodeIndex;
        while (idx >= 0) {
            forwardNodeIndices.push_back(idx);
            idx = allNodes[idx].parentId;
        }
        std::reverse(forwardNodeIndices.begin(), forwardNodeIndices.end());
    }

    output.repoCount = 0;
    for (size_t i = 1; i < forwardNodeIndices.size(); i++) {
        const SearchNode& node = allNodes[forwardNodeIndices[i]];
        int bendId = node.lastBendId;
        if (bendId < 0) continue;

        // Check if repo was triggered at this step
        if (node.state.needsRepo) {
            SequenceAction repoAction;
            repoAction.type = ActionType::REPOSITION;
            repoAction.duration = m_costFn.repoTime(true);
            repoAction.description = "Reposition (reason: " +
                std::to_string(static_cast<int>(node.state.repoReason)) + ")";
            output.actions.push_back(repoAction);
            output.repoCount++;
            output.repoAfterBends.push_back(bendId);
        }

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
