#ifndef MIRA_COLLISIONDETECTION_H
#define MIRA_COLLISIONDETECTION_H

#include "MIRAVector.h"
#include "MIRACollider.h"

// Axis-aligned bounding box
struct AABB
{
    MIRA::Vector3 min; // bottom-left-back
    MIRA::Vector3 max; // top-right-front
};

struct CollisionInfo
{
    MIRA::Vector3 normal; // Collision normal, direction to resolve
    float depth;          // Penetration depth
};

AABB ComputeAABB(const MIRA::Collider& collider);

bool CheckAABBOverlap(const AABB& a, const AABB& b);

// Find the closest point on a line segment [A, B] to a point P
MIRA::Vector3 ClosestPointOnLineSegment(const MIRA::Vector3& A, const MIRA::Vector3& B, const MIRA::Vector3& P);

// Check if a line segment [start, end] overlaps with an AABB
bool CheckLineSegmentAABBOverlap(const MIRA::Vector3& start, const MIRA::Vector3& end, const AABB& aabb);

// Find the clostest point between two line segments
void ClosestPointsBetweenLineSegments(const MIRA::Vector3& A1, const MIRA::Vector3& B1,
    const MIRA::Vector3& A2, const MIRA::Vector3& B2,
    MIRA::Vector3& closest1, MIRA::Vector3& closest2);

// Check for a collision between two colliders
bool CheckCollision(const MIRA::Collider& a, const MIRA::Collider& b, CollisionInfo& info);

// Distance Check
bool SphereSphereCollision(const MIRA::Collider& a, const MIRA::Collider& b, CollisionInfo& info);

// Closest Point projection
bool SphereBoxCollision(const MIRA::Collider& sphere, const MIRA::Collider& box, CollisionInfo& info);

// Separating Axis Theorem (SAT)
bool BoxBoxCollision(const MIRA::Collider& a, const MIRA::Collider& b, CollisionInfo& info);

// Closest point on box to point in space
MIRA::Vector3 GetClosestBoxPoint(const MIRA::Collider& box, const MIRA::Vector3& point);

// Closest point on capsule line segment to sphere center
bool CapsuleSphereCollision(const MIRA::Collider& capsule, const MIRA::Collider& sphere, CollisionInfo& info);

// Closest points between line segments with radius check
bool CapsuleCapsuleCollision(const MIRA::Collider& a, const MIRA::Collider& b, CollisionInfo& info);

// Line segment vs expanded AABB
bool CapsuleBoxCollision(const MIRA::Collider& capsule, const MIRA::Collider& box, CollisionInfo& info);

// Impulse-based resolution with position correction
void ResolveCollision(MIRA::RigidBody& a, MIRA::RigidBody& b, const CollisionInfo& info);
#endif