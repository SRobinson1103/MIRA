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
    return aabb;
}

bool CheckAABBOverlap(const AABB& a, const AABB& b)
{
    return (a.max.x > b.min.x && a.min.x < b.max.x) &&
        (a.max.y > b.min.y && a.min.y < b.max.y) &&
        (a.max.z > b.min.z && a.min.z < b.max.z);
}

bool CheckCollision(const Collider& a, const Collider& b, CollisionInfo& info)
{
    if (a.type == Collider::SPHERE && b.type == Collider::SPHERE)
    {
        return SphereSphereCollision(a, b, info);
    }
    else if (a.type == Collider::SPHERE && b.type == Collider::BOX)
    {
        return SphereBoxCollision(a, b, info);
    }
    else if (a.type == Collider::BOX && b.type == Collider::BOX)
    {
        return BoxBoxCollision(a, b, info);
    }
    return false;
}

bool SphereSphereCollision(const Collider& a, const Collider& b, CollisionInfo& info)
{
    Vector3 delta = b.body->position - a.body->position;
    float distanceSq = delta.Dot(delta);
    float radiusSum = a.GetSphereRadius() + b.GetSphereRadius();
    float radiusSumSq = radiusSum * radiusSum;

    if (distanceSq < radiusSumSq)
    {
        float distance = std::sqrt(distanceSq);
        info.normal = delta / distance;
        info.depth = radiusSum - distance;
        return true;
    }
    return false;
}

bool SphereBoxCollision(const Collider& sphere, const Collider& box, CollisionInfo& info)
{
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

void ResolveCollision(RigidBody& a, RigidBody& b, const CollisionInfo& info)
{
    Vector3 relativeVel = b.velocity - a.velocity;
    float velAlongNormal = relativeVel.Dot(info.normal);

    // Do nothing if objects are separating
    if (velAlongNormal > 0) return;

    // Calculate impulse
    float e = std::min(a.restitution, b.restitution);
    float j = -(1 + e) * velAlongNormal;

    // Handle mass = 0.0f
    float invMassA = (a.mass == 0.0f) ? 0.0f : 1.0f / a.mass;
    float invMassB = (b.mass == 0.0f) ? 0.0f : 1.0f / b.mass;
    j /= (invMassA + invMassB);

    // Apply impulse
    Vector3 impulse = j * info.normal;
    a.velocity = a.velocity - (impulse * invMassA);
    b.velocity = b.velocity + (impulse * invMassB);

    // Position correction 
    const float percent = 0.2f;
    const float slop = 0.01f;
    float correction = std::max(info.depth - slop, 0.0f) / (invMassA + invMassB) * percent;
    Vector3 correctionVec = correction * info.normal;
    a.position = a.position - (correctionVec * invMassA);
    b.position = b.position + (correctionVec * invMassB);
}
