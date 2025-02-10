#ifndef MIRA_BROADPHASEGRID_TESTS_H
#define MIRA_BROADPHASEGRID_TESTS_H

#include "MIRATestFramework.h"
#include "BroadPhaseGrid.h"
#include "MIRACollider.h"
#include "MIRARigidBody.h"

using namespace MIRA;

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
