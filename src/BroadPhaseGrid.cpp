#include "BroadPhaseGrid.h"

#include <algorithm>

using namespace MIRA;

BroadPhaseGrid::BroadPhaseGrid(float cellSize) : cellSize(cellSize) {}

MIRA::Vector3 BroadPhaseGrid::ComputeCellPosition(const MIRA::Vector3& pos) const
{
    if (cellSize <= 0.0f)
    {
        throw std::runtime_error("Cell size must be greater than zero.");
    }
    return MIRA::Vector3
    {
        std::floor(pos.x / cellSize) * cellSize,
        std::floor(pos.y / cellSize) * cellSize,
        std::floor(pos.z / cellSize) * cellSize
    };
}

std::vector<MIRA::Vector3> BroadPhaseGrid::GetOverlappingCells(const MIRA::AABB& bounds) const
{
    int minX = static_cast<int>(std::floor(bounds.min.x / cellSize));
    int minY = static_cast<int>(std::floor(bounds.min.y / cellSize));
    int minZ = static_cast<int>(std::floor(bounds.min.z / cellSize));

    int maxX = static_cast<int>(std::floor(bounds.max.x / cellSize));
    int maxY = static_cast<int>(std::floor(bounds.max.y / cellSize));
    int maxZ = static_cast<int>(std::floor(bounds.max.z / cellSize));

    std::vector<MIRA::Vector3> cells;
    cells.reserve(std::abs((maxX - minX) * (maxY - minY) * (maxZ - minZ) * 3));
    for (int x = minX; x <= maxX; ++x)
    {
        for (int y = minY; y <= maxY; ++y)
        {
            for (int z = minZ; z <= maxZ; ++z)
            {
                cells.push_back(MIRA::Vector3{ x * cellSize, y * cellSize, z * cellSize });
            }
        }
    }

    return cells;
}

float BroadPhaseGrid::ComputeOptimalCellSize(const std::vector<MIRA::Collider>& colliders)
{
    if (colliders.empty()) return 1.0f;

    float totalVolume = 0.0f;
    for (const MIRA::Collider& col : colliders)
    {
        // Volume of AABB
        totalVolume += (col.bounds.max.x - col.bounds.min.x)
                     * (col.bounds.max.y - col.bounds.min.y)
                     * (col.bounds.max.z - col.bounds.min.z);
    }

    float avgVolume = totalVolume / colliders.size();
    float avgSideLength = std::cbrt(avgVolume);

    // Use a fraction of the average side length as the cell size
    return avgSideLength * 0.5f;
}

void BroadPhaseGrid::AddObject(const MIRA::Collider& collider)
{
    std::lock_guard<std::shared_mutex> lock(rwMutex);

    std::vector<MIRA::Vector3> cells = GetOverlappingCells(collider.bounds);
    std::unordered_set<MIRA::Vector3>& objectToCells = objectIDToCells[collider.id];

    for (const MIRA::Vector3& cell : cells)
    {
        grid[cell].objectIDs.push_back(collider.id);
        objectToCells.insert(cell);
    }
}

void BroadPhaseGrid::UpdateObject(const MIRA::Collider& collider)
{
    std::lock_guard<std::shared_mutex> lock(rwMutex);

    std::unordered_set<MIRA::Vector3>& oldCells = objectIDToCells[collider.id];
    std::vector<MIRA::Vector3> newCellVec = GetOverlappingCells(collider.bounds);
    std::unordered_set<MIRA::Vector3> newCells(newCellVec.begin(), newCellVec.end());

    // Remove from cells that are no longer overlapped
    for (const MIRA::Vector3& cellPos : oldCells)
    {
        if (!newCells.count(cellPos))
        {
            Cell& cell = grid[cellPos];
            cell.objectIDs.erase(
                std::remove(cell.objectIDs.begin(), cell.objectIDs.end(), collider.id),
                cell.objectIDs.end()
            );
        }
    }

    // Add to new cells that weren't previously occupied
    for (const auto& cellPos : newCells)
    {
        if (!oldCells.count(cellPos))
        {
            grid[cellPos].objectIDs.push_back(collider.id);
        }
    }

    oldCells = std::move(newCells);  // Update tracked cells
}

void BroadPhaseGrid::RemoveObject(const MIRA::Collider& collider)
{
    std::lock_guard<std::shared_mutex> lock(rwMutex);

    std::vector<MIRA::Vector3> cells = GetOverlappingCells(collider.bounds);
    for (MIRA::Vector3 cellIndex : cells)
    {
        Cell& cell = grid[cellIndex];
        cell.objectIDs.erase(
            std::remove(cell.objectIDs.begin(), cell.objectIDs.end(), collider.id),
            cell.objectIDs.end()
        );
    }
    objectIDToCells.erase(collider.id);
}

std::unordered_set<std::pair<int, int>> BroadPhaseGrid::QueryPotentialCollisionsParallel() const
{
    std::unordered_set<std::pair<int, int>> collisionPairs;
    std::mutex pairsMutex;

    // Acquire shared lock for entire operation
    std::shared_lock<std::shared_mutex> readLock(rwMutex);

    // Create copy of cell positions to avoid iterator invalidation
    std::vector<MIRA::Vector3> cellPositions;
    cellPositions.reserve(grid.size());
    for (const auto& [pos, _] : grid)
    {
        cellPositions.push_back(pos);
    }

    // Parallel processing
    const size_t numThreads = std::thread::hardware_concurrency();
    const size_t chunkSize = (cellPositions.size() + numThreads - 1) / numThreads;
    std::vector<std::thread> threads;
    threads.reserve(numThreads);

    //perform collision checks for chunks of cells, each chunk on its own thread
    for (size_t i = 0; i < numThreads; ++i)
    {
        threads.emplace_back([&, i]()
        {
            std::vector<std::pair<int, int>> localCollisionPairs;
            const size_t start = i * chunkSize;
            const size_t end = std::min(start + chunkSize, cellPositions.size());

            for (size_t j = start; j < end; ++j)
            {
                const Vector3& cellPos = cellPositions[j];
                const Cell& cell = grid.at(cellPos);
                const std::vector<int>& objectIDs = cell.objectIDs;

                // Intra-cell collisions
                for (size_t k = 0; k < objectIDs.size(); ++k)
                {
                    for (size_t l = k + 1; l < objectIDs.size(); ++l)
                    {
                        localCollisionPairs.emplace_back(objectIDs[k], objectIDs[l]);
                    }
                }

                // Inter-cell collisions
                const int x = static_cast<int>(cellPos.x / cellSize);
                const int y = static_cast<int>(cellPos.y / cellSize);
                const int z = static_cast<int>(cellPos.z / cellSize);

                for (int dx = -1; dx <= 1; ++dx)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        for (int dz = -1; dz <= 1; ++dz)
                        {
                            if (dx == 0 && dy == 0 && dz == 0) continue;

                            const MIRA::Vector3 neighborPos
                            {
                                (x + dx) * cellSize,
                                (y + dy) * cellSize,
                                (z + dz) * cellSize
                            };

                            if (grid.count(neighborPos))
                            {
                                const auto& neighborIDs = grid.at(neighborPos).objectIDs;
                                for (const int id1 : objectIDs)
                                {
                                    for (const int id2 : neighborIDs)
                                    {
                                        if (id1 < id2) //avoid duplicate collision pairs
                                        {
                                            localCollisionPairs.emplace_back(id1, id2);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Merge results
            std::lock_guard<std::mutex> lock(pairsMutex);
            collisionPairs.insert(localCollisionPairs.begin(), localCollisionPairs.end());
        });
    }

    for (auto& thread : threads)
    {
        thread.join();
    }
    return collisionPairs;
}
