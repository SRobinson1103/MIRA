#include "CollisionDetection.h"

#include <algorithm>

using namespace MIRA;

AABB ComputeAABB(const Collider& collider)
{
    AABB aabb;
    if (collider.type == Collider::Type::SPHERE)
    {
        aabb.min = collider.body->position - Vector3(collider.GetSphereRadius());
        aabb.max = collider.body->position + Vector3(collider.GetSphereRadius());
    }
    else if (collider.type == Collider::Type::BOX)
    {
        aabb.min = collider.body->position - collider.GetBoxHalfExtents();
        aabb.max = collider.body->position + collider.GetBoxHalfExtents();
    }
    else if (collider.type == Collider::Type::CAPSULE)
    {
        float radius = collider.GetCapsuleRadius();
        float halfHeight = collider.GetCapsuleHeight() * 0.5f;

        // Capsule spans vertically, along Y-axis
        aabb.min = collider.body->position - Vector3(radius, halfHeight + radius, radius);
        aabb.max = collider.body->position + Vector3(radius, halfHeight + radius, radius);
    }
    return aabb;
}

bool CheckAABBOverlap(const AABB& a, const AABB& b)
{
    return (a.max.x > b.min.x && a.min.x < b.max.x) &&
        (a.max.y > b.min.y && a.min.y < b.max.y) &&
        (a.max.z > b.min.z && a.min.z < b.max.z);
}

Vector3 ClosestPointOnLineSegment(const Vector3& A, const Vector3& B, const Vector3& P)
{
    Vector3 AB = B - A;
    float t = (P - A).Dot(AB) / AB.Dot(AB);
    t = fminf(fmaxf(t, 0.0f), 1.0f);
    return A + AB * t;
}

bool CheckLineSegmentAABBOverlap(const Vector3& start, const Vector3& end, const AABB& aabb)
{
    // Represent the segment parametrically as start + t * (end - start), t E [0,1].
    // Compute intersection intervals with each AABB slab (X, Y, Z).
    // Track the overlap of all intervals.
    // Return true if all intervals overlap (segment intersects AABB).

    if (start.x > aabb.max.x && end.x > aabb.max.x) return false;
    if (start.x < aabb.min.x && end.x < aabb.min.x) return false;
    if (start.y > aabb.max.y && end.y > aabb.max.y) return false;
    if (start.y < aabb.min.y && end.y < aabb.min.y) return false;
    if (start.z > aabb.max.z && end.z > aabb.max.z) return false;
    if (start.z < aabb.min.z && end.z < aabb.min.z) return false;

    Vector3 dir = end - start;
    float t_min = 0.0f;
    float t_max = 1.0f;

    // Test each axis (X, Y, Z)
    for (int axis = 0; axis < 3; ++axis)
    {
        if (std::abs(dir[axis]) < 1e-6) { // Parallel to slab
            if (start[axis] < aabb.min[axis] || start[axis] > aabb.max[axis])
            {
                return false;
            }
        }
        else
        {
            float inv_dir = 1.0f / dir[axis];
            float t1 = (aabb.min[axis] - start[axis]) * inv_dir;
            float t2 = (aabb.max[axis] - start[axis]) * inv_dir;

            t_min = std::max(t_min, std::min(t1, t2));
            t_max = std::min(t_max, std::max(t1, t2));
        }
    }

    return t_min <= t_max;
}

void ClosestPointsBetweenLineSegments(const Vector3& A1, const Vector3& B1,
    const Vector3& A2, const Vector3& B2,
    Vector3& closest1, Vector3& closest2)
{
    // Finds the closest points between two line segments [A1, B1] and [A2, B2]:
    // Compute the direction vectors and relative position of the segments.
    // Solve for the closest points using parametric equations.
    // Clamp the parameters to the segment ranges.
    // Check all endpoint pairs to ensure the minimal distance is found.

    Vector3 d1 = B1 - A1;
    Vector3 d2 = B2 - A2;
    Vector3 r = A1 - A2;

    float a = d1.Dot(d1);
    float b = d1.Dot(d2);
    float c = d2.Dot(d2);
    float d = d1.Dot(r);
    float e = d2.Dot(r);
    float denom = a * c - b * b;

    float s = 0.0f, t = 0.0f;

    if (denom != 0.0f)
    {
        s = (b * e - c * d) / denom;
        t = (a * e - b * d) / denom;
    }
    else
    {
        // Handle parallel segments by projecting endpoints
        s = 0.0f;
        t = (b > c ? d / b : e / c);
        t = std::clamp(t, 0.0f, 1.0f);

        // Project closest point from segment2 to segment1
        Vector3 closestOn2 = A2 + d2 * t;
        s = std::clamp((d1.Dot(closestOn2 - A1)) / a, 0.0f, 1.0f);
    }

    // Compute initial closest points
    closest1 = A1 + d1 * std::clamp(s, 0.0f, 1.0f);
    closest2 = A2 + d2 * std::clamp(t, 0.0f, 1.0f);

    // Check all endpoint combinations
    float bestDistSq = (closest2 - closest1).Dot(closest2 - closest1);
    auto checkPair = [&](const Vector3& p1, const Vector3& p2)
    {
        Vector3 delta = p2 - p1;
        float distSq = delta.Dot(delta);
        if (distSq < bestDistSq)
        {
            bestDistSq = distSq;
            closest1 = p1;
            closest2 = p2;
        }
    };

    checkPair(A1, A2);
    checkPair(A1, B2);
    checkPair(B1, A2);
    checkPair(B1, B2);
}

bool CheckCollision(const Collider& a, const Collider& b, CollisionInfo& info)
{
    // Define collision type matrix
    constexpr int NUM_TYPES = 3;
    using CollisionHandler = bool(*)(const Collider&, const Collider&, CollisionInfo&);
    static const CollisionHandler handlers[NUM_TYPES][NUM_TYPES] =
    {
        /* SPHERE */ {SphereSphereCollision, SphereBoxCollision, CapsuleSphereCollision},
        /* BOX    */ {nullptr, BoxBoxCollision, CapsuleBoxCollision},
        /* CAPSULE*/ {nullptr, nullptr, CapsuleCapsuleCollision}
    };

    // Get ordered indices
    const int a_idx = static_cast<int>(a.type);
    const int b_idx = static_cast<int>(b.type);

    // Check if we need to swap order
    if (a_idx > b_idx)
    {
        const bool result = CheckCollision(b, a, info);
        info.normal = -info.normal;
        return result;
    }

    // Get handler from matrix
    if (auto handler = handlers[a_idx][b_idx])
    {
        return handler(a, b, info);
    }

    return false;
}

bool SphereSphereCollision(const Collider& a, const Collider& b, CollisionInfo& info)
{
    // Calculate vector between centers
    // Compare squared distance to squared sum of radii
    // Calculate penetration depth and normal
    // Handle perfect overlap edge case
    Vector3 delta = b.body->position - a.body->position;
    float distanceSq = delta.Dot(delta);
    float radiusSum = a.GetSphereRadius() + b.GetSphereRadius();

    if (distanceSq > radiusSum * radiusSum) return false;

    const float distance = std::sqrt(distanceSq);
    info.normal = distance > 0 ? delta / distance : Vector3(1, 0, 0);
    info.depth = radiusSum - distance;
    return true;
}

bool SphereBoxCollision(const Collider& sphere, const Collider& box, CollisionInfo& info)
{
    // Find closest point on box to sphere center (clamping)
    // Check distance between sphere center and clamped point
    // Handle containment case separately
    // Determine penetration axis for containment resolution
    const Vector3& sphereCenter = sphere.body->position;
    const Vector3& boxCenter = box.body->position;
    const Vector3& boxHalf = box.GetBoxHalfExtents();
    const float radius = sphere.GetSphereRadius();

    // Compute box bounds
    const Vector3 boxMin = boxCenter - boxHalf;
    const Vector3 boxMax = boxCenter + boxHalf;

    // Find closest point on box to sphere
    Vector3 closestPoint;
    closestPoint.x = std::clamp(sphereCenter.x, boxMin.x, boxMax.x);
    closestPoint.y = std::clamp(sphereCenter.y, boxMin.y, boxMax.y);
    closestPoint.z = std::clamp(sphereCenter.z, boxMin.z, boxMax.z);

    Vector3 delta = sphereCenter - closestPoint;
    float distanceSq = delta.Dot(delta);

    if (distanceSq > radius * radius) return false;

    if (distanceSq > 0.0f)
    {
        // Sphere outside box
        float distance = std::sqrt(distanceSq);
        info.normal = delta / distance;
        info.depth = radius - distance;
    }
    else
    {
        // Sphere inside box: find minimal penetration axis
        Vector3 penetration = boxHalf - (sphereCenter - boxCenter).Abs();
        int axis = (penetration.x < penetration.y) ?
            ((penetration.x < penetration.z) ? 0 : 2) :
            ((penetration.y < penetration.z) ? 1 : 2);

        info.normal = Vector3(0.0f);
        info.normal[axis] = (sphereCenter[axis] > boxCenter[axis]) ? 1.0f : -1.0f;
        info.depth = radius + penetration[axis];
    }
    return true;
}

bool BoxBoxCollision(const Collider& a, const Collider& b, CollisionInfo& info)
{
    // Calculate penetration depth on all axes
    // Early exit on any axis with no overlap
    // Find minimum penetration axis
    // Construct collision normal from penetration axis
    const Vector3 delta = b.body->position - a.body->position;
    const Vector3 penetration = (a.GetBoxHalfExtents() + b.GetBoxHalfExtents()) - delta.Abs();

    // Early exit if any axis has no overlap
    if (penetration.x < 0 || penetration.y < 0 || penetration.z < 0) return false;

    // Find axis with minimal penetration
    int axis = 0;
    float minPenetration = penetration.x;
    if (penetration.y < minPenetration)
    {
        axis = 1;
        minPenetration = penetration.y;
    }
    if (penetration.z < minPenetration)
    {
        axis = 2;
    }

    info.depth = penetration[axis];
    info.normal = Vector3(0.0f);
    info.normal[axis] = (delta[axis] > 0) ? 1.0f : -1.0f;
    return true;
}

bool CapsuleSphereCollision(const MIRA::Collider& capsule, const MIRA::Collider& sphere, CollisionInfo& info)
{
    // Treat capsule as line segment with radius
    // Find closest point on capsule line to sphere center
    // Check sphere-to-point distance against combined radii
    // Calculate collision normal from closest point
    
    // Extract data
    Vector3 capPos = capsule.body->position;
    float capRadius = capsule.GetCapsuleRadius();
    float capHalfHeight = capsule.GetCapsuleHeight() * 0.5f;

    Vector3 spherePos = sphere.body->position;
    float sphereRadius = sphere.GetSphereRadius();

    // Capsule line segment endpoints (vertical)
    Vector3 capTop = capPos + Vector3(0, capHalfHeight, 0);
    Vector3 capBottom = capPos - Vector3(0, capHalfHeight, 0);

    // Closest point on capsule line to sphere center
    Vector3 closestOnCapsule = ClosestPointOnLineSegment(capTop, capBottom, spherePos);

    // Distance between closest point and sphere center
    Vector3 delta = spherePos - closestOnCapsule;
    float distance = delta.Magnitude();
    float totalRadius = capRadius + sphereRadius;

    if (distance < totalRadius)
    {
        info.normal = delta.Normalized();
        info.depth = totalRadius - distance;
        return true;
    }
    return false;
}

bool CapsuleCapsuleCollision(const MIRA::Collider& a, const MIRA::Collider& b, CollisionInfo& info)
{
    // Find closest points between two line segments
    // Check distance between closest points against sum of radii
    // Handle parallel/colinear edge cases
    // Construct collision normal from closest points

    // Capsule A data
    Vector3 aPos = a.body->position;
    float aRadius = a.GetCapsuleRadius();
    float aHalfHeight = a.GetCapsuleHeight() * 0.5f;
    Vector3 aTop = aPos + Vector3(0, aHalfHeight, 0);
    Vector3 aBottom = aPos - Vector3(0, aHalfHeight, 0);

    // Capsule B data
    Vector3 bPos = b.body->position;
    float bRadius = b.GetCapsuleRadius();
    float bHalfHeight = b.GetCapsuleHeight() * 0.5f;
    Vector3 bTop = bPos + Vector3(0, bHalfHeight, 0);
    Vector3 bBottom = bPos - Vector3(0, bHalfHeight, 0);

    // Find closest points between the two line segments
    Vector3 closestA, closestB;
    ClosestPointsBetweenLineSegments(aTop, aBottom, bTop, bBottom, closestA, closestB);

    // Distance between closest points
    Vector3 delta = closestB - closestA;
    float distance = delta.Magnitude();
    float totalRadius = aRadius + bRadius;

    if (distance < totalRadius)
    {
        info.normal = (distance > 1e-6f) ? delta.Normalized() : Vector3(0, 1, 0);
        info.depth = totalRadius - distance;
        return true;
    }
    return false;
}

Vector3 GetClosestBoxPoint(const Collider& box, const Vector3& point)
{
    const Vector3& center = box.body->position;
    const Vector3& half = box.GetBoxHalfExtents();

    return Vector3(
        std::clamp(point.x, center.x - half.x, center.x + half.x),
        std::clamp(point.y, center.y - half.y, center.y + half.y),
        std::clamp(point.z, center.z - half.z, center.z + half.z)
    );
}

bool CapsuleBoxCollision(const Collider& capsule, const Collider& box, CollisionInfo& info)
{
    // Expand the box's AABB by the capsule's radius.
    // Check if the capsule's line segment intersects the expanded AABB.
    // Find the closest point on the capsule to the box.
    // Treat the closest point as a sphere and check for collision with the box.

    // Capsule data
    const float capRadius = capsule.GetCapsuleRadius();
    const float halfHeight = capsule.GetCapsuleHeight() * 0.5f;
    const Vector3 capPos = capsule.body->position;
    const Vector3 capTop = capPos + Vector3(0, halfHeight, 0);
    const Vector3 capBottom = capPos - Vector3(0, halfHeight, 0);

    // Box data
    const Vector3& boxHalf = box.GetBoxHalfExtents();
    const Vector3& boxCenter = box.body->position;

    // Expand box's AABB by capsule radius
    const AABB expandedBoxAABB = {
        boxCenter - boxHalf - Vector3(capRadius),
        boxCenter + boxHalf + Vector3(capRadius)
    };

    // Check if the capsule's line segment intersects the expanded AABB
    if (!CheckLineSegmentAABBOverlap(capTop, capBottom, expandedBoxAABB))
        return false;

    // Find closest point on capsule to box center
    const Vector3 closestOnCapsule = ClosestPointOnLineSegment(capTop, capBottom, boxCenter);

    // Treat closest point as a sphere and check against box
    const Vector3 delta = boxCenter - closestOnCapsule;
    const float distance = delta.Magnitude();
    const float totalRadius = capRadius + boxHalf.x; // Use box's half-extent for radius

    if (distance < totalRadius)
    {
        info.normal = delta.Normalized();
        info.depth = totalRadius - distance;
        return true;
    }

    return false;
}

void ResolveCollision(RigidBody& a, RigidBody& b, const CollisionInfo& info)
{
    // Calculate relative velocity
    // Apply impulse based on restitution and mass
    // Position correction to prevent sinking
    // Handle infinite mass (static objects)
    const Vector3 relativeVel = b.velocity - a.velocity;
    const float velAlongNormal = relativeVel.Dot(info.normal);

    if (velAlongNormal > 0) return;

    // Calculate impulse
    float e = std::min(a.restitution, b.restitution);
    float j = -(1 + e) * velAlongNormal;

    // Handle infinite mass
    float invMassA = (a.mass == 0.0f) ? 0.0f : 1.0f / a.mass;
    float invMassB = (b.mass == 0.0f) ? 0.0f : 1.0f / b.mass;
    j /= (invMassA + invMassB);

    // Apply impulse
    Vector3 impulse = j * info.normal;
    a.velocity = a.velocity - impulse * invMassA;
    b.velocity = b.velocity + impulse * invMassB;

    // Position correction
    const float percent = 0.2f; // Penetration percentage to resolve
    const float slop = 0.01f;   // Tolerance to prevent jitter
    float correction = (std::max(info.depth - slop, 0.0f) / (invMassA + invMassB)) * percent;

    // Apply position correction
    Vector3 correctionVec = correction * info.normal;
    a.position = a.position - correctionVec * invMassA;
    b.position = b.position + correctionVec * invMassB;
}
