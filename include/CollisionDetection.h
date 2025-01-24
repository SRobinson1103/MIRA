#ifndef MIRA_AABB_H
#define MIRA_AABB_H

#include "MIRAVector.h"
#include "MIRACollider.h"

struct AABB
{
    MIRA::Vector3 min;
    MIRA::Vector3 max;
};

AABB ComputeAABB(const MIRA::Collider& collider);

bool CheckAABBOverlap(const AABB& a, const AABB& b);

struct CollisionInfo
{
    MIRA::Vector3 normal; // Collision normal (direction to resolve)
    float depth;          // Penetration depth
};

bool CheckCollision(const MIRA::Collider& a, const MIRA::Collider& b, CollisionInfo& info);

bool SphereSphereCollision(const MIRA::Collider& a, const MIRA::Collider& b, CollisionInfo& info);

bool SphereBoxCollision(const MIRA::Collider& sphere, const MIRA::Collider& box, CollisionInfo& info);

bool BoxBoxCollision(const MIRA::Collider& a, const MIRA::Collider& b, CollisionInfo& info);

void ResolveCollision(MIRA::RigidBody& a, MIRA::RigidBody& b, const CollisionInfo& info);
#endif