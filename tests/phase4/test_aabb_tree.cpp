#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase4/aabb_tree.h"

using namespace openpanelcam::phase4;
using Catch::Approx;

// ===== AABBTree Basic Tests =====

TEST_CASE("AABBTree starts empty", "[phase4][aabb_tree]") {
    AABBTree tree;
    REQUIRE(tree.size() == 0);
}

TEST_CASE("AABBTree insert single element", "[phase4][aabb_tree]") {
    AABBTree tree;
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);

    REQUIRE(tree.size() == 1);
    AABB root = tree.getRootBounds();
    REQUIRE(root.minX == 0.0);
    REQUIRE(root.maxX == 10.0);
}

TEST_CASE("AABBTree insert multiple elements", "[phase4][aabb_tree]") {
    AABBTree tree;
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);
    tree.insert(AABB(20, 20, 20, 30, 30, 30), 1);
    tree.insert(AABB(50, 50, 50, 60, 60, 60), 2);

    REQUIRE(tree.size() == 3);

    // Root bounds should encompass all
    AABB root = tree.getRootBounds();
    REQUIRE(root.minX <= 0.0);
    REQUIRE(root.maxX >= 60.0);
    REQUIRE(root.minY <= 0.0);
    REQUIRE(root.maxY >= 60.0);
}

TEST_CASE("AABBTree query finds overlapping", "[phase4][aabb_tree]") {
    AABBTree tree;
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);
    tree.insert(AABB(20, 20, 20, 30, 30, 30), 1);
    tree.insert(AABB(50, 50, 50, 60, 60, 60), 2);

    std::vector<int> results;

    // Query overlapping with bend 0
    tree.query(AABB(-5, -5, -5, 5, 5, 5), results);
    REQUIRE(results.size() == 1);
    REQUIRE(results[0] == 0);
}

TEST_CASE("AABBTree query finds multiple overlapping", "[phase4][aabb_tree]") {
    AABBTree tree;
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);
    tree.insert(AABB(5, 5, 5, 15, 15, 15), 1);
    tree.insert(AABB(100, 100, 100, 200, 200, 200), 2);

    std::vector<int> results;

    // Query overlapping with both bend 0 and 1
    tree.query(AABB(3, 3, 3, 12, 12, 12), results);
    REQUIRE(results.size() == 2);
}

TEST_CASE("AABBTree query no results for distant query", "[phase4][aabb_tree]") {
    AABBTree tree;
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);
    tree.insert(AABB(20, 20, 20, 30, 30, 30), 1);

    std::vector<int> results;
    tree.query(AABB(500, 500, 500, 600, 600, 600), results);
    REQUIRE(results.empty());
}

TEST_CASE("AABBTree remove element", "[phase4][aabb_tree]") {
    AABBTree tree;
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);
    tree.insert(AABB(20, 20, 20, 30, 30, 30), 1);

    REQUIRE(tree.size() == 2);

    bool removed = tree.remove(0);
    REQUIRE(removed == true);
    REQUIRE(tree.size() == 1);

    // Query should not find removed bend
    std::vector<int> results;
    tree.query(AABB(-5, -5, -5, 5, 5, 5), results);
    REQUIRE(results.empty());
}

TEST_CASE("AABBTree remove nonexistent returns false", "[phase4][aabb_tree]") {
    AABBTree tree;
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);

    REQUIRE(tree.remove(99) == false);
    REQUIRE(tree.size() == 1);
}

TEST_CASE("AABBTree clear empties tree", "[phase4][aabb_tree]") {
    AABBTree tree;
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);
    tree.insert(AABB(20, 20, 20, 30, 30, 30), 1);

    tree.clear();
    REQUIRE(tree.size() == 0);

    std::vector<int> results;
    tree.query(AABB(-100, -100, -100, 100, 100, 100), results);
    REQUIRE(results.empty());
}

TEST_CASE("AABBTree query empty tree", "[phase4][aabb_tree]") {
    AABBTree tree;
    std::vector<int> results;
    tree.query(AABB(0, 0, 0, 10, 10, 10), results);
    REQUIRE(results.empty());
}

TEST_CASE("AABBTree empty root bounds", "[phase4][aabb_tree]") {
    AABBTree tree;
    AABB root = tree.getRootBounds();
    REQUIRE(root.volume() == 0.0);
}

TEST_CASE("AABBTree SAH picks nearby sibling", "[phase4][aabb_tree]") {
    AABBTree tree;
    // Insert two distant clusters
    tree.insert(AABB(0, 0, 0, 10, 10, 10), 0);
    tree.insert(AABB(5, 5, 5, 15, 15, 15), 1);
    tree.insert(AABB(1000, 1000, 1000, 1010, 1010, 1010), 2);

    // Query near cluster 1 should only find bends 0,1
    std::vector<int> results;
    tree.query(AABB(-1, -1, -1, 8, 8, 8), results);

    bool found0 = false, found1 = false, found2 = false;
    for (int id : results) {
        if (id == 0) found0 = true;
        if (id == 1) found1 = true;
        if (id == 2) found2 = true;
    }
    REQUIRE(found0 == true);
    REQUIRE(found1 == true);
    REQUIRE(found2 == false);
}
