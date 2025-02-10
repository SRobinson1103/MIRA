#ifndef MIRA_BROADPHASEGRID_TESTS_H
#define MIRA_BROADPHASEGRID_TESTS_H

#include "MIRATestFramework.h"
#include "BroadPhaseGrid.h"
#include "MIRACollider.h"
#include "MIRARigidBody.h"

using namespace MIRA;

TEST_CASE(ComputeCellPosition_CalculatesCorrectCell)
{
    BroadPhaseGrid grid(2.0f);

    // Basic case
    MIRA::Vector3 pos1{ 1.5f, 3.0f, 4.9f };
    MIRA::Vector3 cell1 = grid.ComputeCellPosition(pos1);
    ASSERT_EQUAL(cell1.x, 0.0f);
    ASSERT_EQUAL(cell1.y, 2.0f);
    ASSERT_EQUAL(cell1.z, 4.0f);

    // Negative coordinates
    MIRA::Vector3 pos2{ -1.0f, -3.5f, -5.1f };
    MIRA::Vector3 cell2 = grid.ComputeCellPosition(pos2);
    ASSERT_EQUAL(cell2.x, -2.0f);
    ASSERT_EQUAL(cell2.y, -4.0f);
    ASSERT_EQUAL(cell2.z, -6.0f);

    // Edge case at cell boundary
    MIRA::Vector3 pos3{ 2.0f, 4.0f, 6.0f };
    MIRA::Vector3 cell3 = grid.ComputeCellPosition(pos3);
    ASSERT_EQUAL(cell3.x, 2.0f);
    ASSERT_EQUAL(cell3.y, 4.0f);
    ASSERT_EQUAL(cell3.z, 6.0f);
}

TEST_CASE(GetOverlappingCells_IdentifiesAllRelevantCells)
{
    BroadPhaseGrid grid(2.0f);

    // AABB spanning 8 cells
    MIRA::AABB bounds1{
        {1.5f, 1.5f, 1.5f},
        {3.5f, 3.5f, 3.5f}
    };
    auto cells1 = grid.GetOverlappingCells(bounds1);
    ASSERT_EQUAL(cells1.size(), 8);

    // Single cell
    MIRA::AABB bounds2{
        {0.1f, 0.1f, 0.1f},
        {1.9f, 1.9f, 1.9f}
    };
    auto cells2 = grid.GetOverlappingCells(bounds2);
    ASSERT_EQUAL(cells2.size(), 1);

    // Negative coordinates
    MIRA::AABB bounds3{
        {-3.5f, -3.5f, -3.5f},
        {-0.5f, -0.5f, -0.5f}
    };
    auto cells3 = grid.GetOverlappingCells(bounds3);
    ASSERT_EQUAL(cells3.size(), 8);
}

TEST_CASE(BroadPhaseGrid_Constructor_SetsCellSize)
{
    BroadPhaseGrid grid1(5.0f);
    // Verify through behavior
    MIRA::Vector3 pos{ 1.0f, 1.0f, 1.0f };
    auto cell = grid1.ComputeCellPosition(pos);
    ASSERT_EQUAL(cell.x, 0.0f);
    ASSERT_EQUAL(cell.y, 0.0f);
    ASSERT_EQUAL(cell.z, 0.0f);

    BroadPhaseGrid grid2(1.0f);
    auto cell2 = grid2.ComputeCellPosition({ 1.5f, 1.5f, 1.5f });
    ASSERT_EQUAL(cell2.x, 1.0f);
}

TEST_CASE(ComputeOptimalCellSize_CalculatesReasonableSize)
{
    std::vector<MIRA::Collider> colliders;
    BroadPhaseGrid grid(1.0f); // Default size

    // Test with empty colliders
    float size1 = grid.ComputeOptimalCellSize(colliders);
    ASSERT_FLOAT_EQUAL(size1, 1.0f); // Should return default

    // Add colliders
    RigidBody rb1, rb2, rb3;
    MIRA::AABB aabb1{ .min = {0,0,0}, .max = {2,2,2} };
    MIRA::AABB aabb2{ .min = {0,0,0}, .max = {4,4,4} };
    MIRA::AABB aabb3{ .min = {0,0,0}, .max = {6,6,6} };
    colliders.emplace_back(1, Collider::Type::BOX, &rb1, aabb1); // Volume 8
    colliders.emplace_back(2, Collider::Type::BOX, &rb2, aabb2); // Volume 64
    colliders.emplace_back(3, Collider::Type::BOX, &rb3, aabb3); // Volume 216

    float avgVolume = (8 + 64 + 216) / 3.0f; // 96
    float expectedSize = static_cast<float>(std::cbrt(96) * 0.5f); // ~4.58 * 0.5 = ~2.29
    float actualSize = grid.ComputeOptimalCellSize(colliders);
    ASSERT_NEAR(actualSize, expectedSize, 0.01f);
}

TEST_CASE(AddObject_RegistersAllOverlappingCells)
{
    BroadPhaseGrid grid(2.0f);
    RigidBody rb;
    MIRA::AABB aabb1{ .min = {1.5f,1.5f,1.5f}, .max = {3.5f,3.5f,3.5f} };
    MIRA::Collider col(1, Collider::Type::BOX, &rb, aabb1);

    grid.AddObject(col);
    auto pairs = grid.QueryPotentialCollisionsParallel();
    ASSERT_TRUE(pairs.empty()); // No collisions yet

    // Verify through update check
    MIRA::AABB aabb2{ .min = {3.5f,3.5f,3.5f}, .max = {4.0f,4.0f,4.0f} };
    MIRA::Collider col2(2, Collider::Type::BOX, &rb, aabb2);
    grid.AddObject(col2);
    pairs = grid.QueryPotentialCollisionsParallel();
    ASSERT_EQUAL(pairs.size(), 1);
}

TEST_CASE(UpdateObject_MovesBetweenCells)
{
    BroadPhaseGrid grid(2.0f);
    RigidBody rb;

    // Initial collider
    MIRA::AABB aabb1{ .min = {10.5f,10.5f,10.5f}, .max = {10.5f,10.5f,10.5f} };
    MIRA::Collider col1(1, Collider::Type::BOX, &rb, aabb1);
    grid.AddObject(col1);

    // New position
    MIRA::AABB aabb2{ .min = {2.5f,2.5f,2.5f}, .max = {3.5f,3.5f,3.5f} };
    MIRA::Collider col2(1, Collider::Type::BOX, &rb, aabb2);
    grid.UpdateObject(col2);

    // Verify old cells are empty
    MIRA::AABB testaabb{ .min = {0.0f,0.0f,0.0f}, .max = {2.0f,2.0f,2.0f} };
    MIRA::Collider testCol(99, Collider::Type::BOX, &rb, testaabb);
    grid.AddObject(testCol);
    auto pairs = grid.QueryPotentialCollisionsParallel();
    ASSERT_EQUAL(pairs.size(), 1); // Original object moved close
}

TEST_CASE(RemoveObject_ClearsFromAllCells)
{
    BroadPhaseGrid grid(2.0f);
    RigidBody rb;
    MIRA::AABB aabb{ .min = {1.0f,1.0f,1.0f}, .max = {3.0f,3.0f,3.0f} };
    MIRA::Collider col(1, Collider::Type::BOX, &rb, aabb);

    grid.AddObject(col);
    grid.RemoveObject(col);

    // Add test object that would collide if original remained
    MIRA::AABB testaabb{ .min = {1.5f,1.5f,1.5f}, .max = {2.5f,2.5f,2.5f} };
    MIRA::Collider testCol(2, Collider::Type::BOX, &rb, testaabb);
    grid.AddObject(testCol);
    auto pairs = grid.QueryPotentialCollisionsParallel();
    ASSERT_TRUE(pairs.empty());
}

TEST_CASE(RemoveObject_InvalidObjectNoCrash)
{
    BroadPhaseGrid grid(2.0f);
    RigidBody rb;
    MIRA::AABB aabb{ .min = {1.0f,1.0f,1.0f}, .max = {2.0f,2.0f,2.0f} };
    MIRA::Collider col(1, Collider::Type::BOX, &rb, aabb);

    // Remove non-existent object
    grid.RemoveObject(col); // Should handle gracefully, No Crash = succes
}

TEST_CASE(BroadPhaseGrid_AddObject_AddsToCorrectCells)
{
    BroadPhaseGrid grid(2.0f);

    RigidBody rb1;
    AABB aabb1{ {0.5f,0.5f,0.5f}, {1.5f,1.5f,1.5f} };
    Collider col1(1, Collider::Type::BOX, &rb1, aabb1);

    grid.AddObject(col1);

    std::unordered_set<std::pair<int, int>> pairs = grid.QueryPotentialCollisionsParallel();
    ASSERT_TRUE(pairs.empty());
}

TEST_CASE(BroadPhaseGrid_UpdateObject_MovesBetweenCells)
{
    BroadPhaseGrid grid(2.0f);
    RigidBody rb1;

    AABB initialAABB{ {0.5f,0.5f,0.5f}, {1.5f,1.5f,1.5f} };
    Collider col1(1, Collider::Type::BOX, &rb1, initialAABB);
    grid.AddObject(col1);

    AABB updatedAABB{ {2.5f,2.5f,2.5f}, {3.5f,3.5f,3.5f} };
    Collider updatedCol1(1, Collider::Type::BOX, &rb1, updatedAABB);

    grid.UpdateObject(updatedCol1);

    std::unordered_set<std::pair<int, int>> pairs = grid.QueryPotentialCollisionsParallel();
    ASSERT_TRUE(pairs.empty());
}

TEST_CASE(BroadPhaseGrid_QueryFindsIntraCellCollisions)
{
    BroadPhaseGrid grid(5.0f);
    RigidBody rb1, rb2;

    AABB aabb1{ {1.0f,1.0f,1.0f}, {2.0f,2.0f,2.0f} };
    AABB aabb2{ {1.5f,1.5f,1.5f}, {2.5f,2.5f,2.5f} };
    Collider col1(1, Collider::Type::BOX, &rb1, aabb1);
    Collider col2(2, Collider::Type::BOX, &rb2, aabb2);

    grid.AddObject(col1);
    grid.AddObject(col2);

    std::unordered_set<std::pair<int, int>> pairs = grid.QueryPotentialCollisionsParallel();
    ASSERT_EQUAL(pairs.size(), 1);
    ASSERT_TRUE((*pairs.begin() == std::pair{ 1,2 }) || (*pairs.begin() == std::pair{ 2,1 }));
}

TEST_CASE(BroadPhaseGrid_QueryFindsInterCellCollisions)
{
    BroadPhaseGrid grid(2.0f);
    RigidBody rb1, rb2;

    AABB aabb1{ {1.5f,1.5f,1.5f}, {2.0f,2.0f,2.0f} };
    AABB aabb2{ {2.1f,2.1f,2.1f}, {2.5f,2.5f,2.5f} };
    Collider col1(1, Collider::Type::BOX, &rb1, aabb1);
    Collider col2(2, Collider::Type::BOX, &rb2, aabb2);

    grid.AddObject(col1);
    grid.AddObject(col2);

    std::unordered_set<std::pair<int, int>> pairs = grid.QueryPotentialCollisionsParallel();
    ASSERT_EQUAL(pairs.size(), 1);
}

#endif
