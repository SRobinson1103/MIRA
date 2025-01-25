#ifndef COLLISION_DETECTION2D_H
#define COLLISION_DETECTION2D_H

#include "MIRACollider2D.h"

struct AABB2D
{
    float min_x, max_x;
    float min_y, max_y;
};

struct CollisionInfo2D
{
    MIRA::Vector2 normal; // Collision normal, direction to resolve
    float depth;          // Penetration depth
};

AABB2D ComputeAABB2D(const MIRA::Collider2D& collider);
bool CheckAABBOverlap2D(const AABB2D& a, const AABB2D& b);

bool CheckCollision(const MIRA::Collider2D& a, const MIRA::Collider2D& b, CollisionInfo2D& info);

bool CircleCircleCollision(const MIRA::Collider2D& a, const MIRA::Collider2D& b, CollisionInfo2D& info);
bool CircleRectCollision(const MIRA::Collider2D& circle, const MIRA::Collider2D& rect, CollisionInfo2D& info);
bool RectRectCollision(const MIRA::Collider2D& a, const MIRA::Collider2D& b, CollisionInfo2D& info);
#endif
