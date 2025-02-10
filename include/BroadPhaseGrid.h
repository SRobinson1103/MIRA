#ifndef BROAD_PHASE_GRID_H
#define BROAD_PHASE_GRID_H

#include "MIRACollider.h"

#include <cmath>
#include <mutex>
#include <thread>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Required in order to use std::pair<int, int> as key in unordered_Set
template<>
struct std::hash<std::pair<int, int>>
{
    size_t operator()(const std::pair<int, int>& p) const
    {
        return static_cast<size_t>(p.first) ^ (static_cast<size_t>(p.second) << 16);
    }
};

// Required in order to use Vector3 as key in unordered_map
template<>
struct std::hash<MIRA::Vector3>
{
    size_t operator()(const MIRA::Vector3& pos) const
    {
        int x = static_cast<int>(std::floor(pos.x));
        int y = static_cast<int>(std::floor(pos.y));
        int z = static_cast<int>(std::floor(pos.z));

        // Combine values using bitwise operations
        return (static_cast<size_t>(x) * 73856093 ^ static_cast<size_t>(y) * 19349663 ^ static_cast<size_t>(z) * 83492791);
    }
};

class BroadPhaseGrid
{
private:

    struct Cell
    {
        std::vector<int> objectIDs; // IDs of objects in this cell
    };

    float cellSize; // Size of each grid cel
    std::unordered_map<MIRA::Vector3, Cell> grid; // Hash map to store cells 

    mutable std::shared_mutex rwMutex;
    std::unordered_map<int, std::unordered_set<MIRA::Vector3>> objectIDToCells; // Map object ID to its current cell position for faster lookup

public:
    BroadPhaseGrid(float cellSize);

    // Helper function to compute the grid index for a point
    MIRA::Vector3 ComputeCellPosition(const MIRA::Vector3& pos) const;

    // Get all cells that an AABB overlaps
    std::vector<MIRA::Vector3> GetOverlappingCells(const MIRA::AABB& bounds) const;

    // Helper function to compute the grid index for a point
    float ComputeOptimalCellSize(const std::vector<MIRA::Collider>& colliders);

    // Add an object to the grid
    void AddObject(const MIRA::Collider& collider);

    // Update an object's position in the grid
    void UpdateObject(const MIRA::Collider& obj);

    // Remove an object from the grid
    void RemoveObject(const MIRA::Collider& collider);

    // Query potential collision pairs
    std::unordered_set<std::pair<int, int>> QueryPotentialCollisionsParallel() const;
};

#endif
