#ifndef MIRA_COLLIDER_H
#define MIRA_COLLIDER_H

#include "MIRARigidBody.h"

#include <cassert>

namespace MIRA
{
// Avoids polymorphism for slight performance gains
class Collider
{
public:
    enum Type { SPHERE, BOX, CAPSULE };
    Type type;
    RigidBody* body; // Associated rigid body

    Collider(Type type, RigidBody* body);

private:
    union
    {
        struct { float radius; } sphere;
        struct { Vector3 halfExtents; } box;
        struct { float radius;
                 float height; /*height of the spine*/ } capsule;
    };

public:
    // Debug only accessors and setters with validation
#if !defined(NDEBUG)
    float GetSphereRadius() const
    {
        assert(type == SPHERE && "Collider is not a sphere!");
        return sphere.radius;
    }

    const Vector3& GetBoxHalfExtents() const
    {
        assert(type == BOX && "Collider is not a box!");
        return box.halfExtents;
    }

    float GetCapsuleRadius() const
    {
        assert(type == CAPSULE && "Collider is not a capsule!");
        return capsule.radius;
    }

    float GetCapsuleHeight() const
    {
        assert(type == CAPSULE && "Collider is not a capsule!");
        return capsule.height;
    }

    void SetSphereRadius(float radius)
    {
        assert(type == SPHERE && "Collider is not a sphere!");
        sphere.radius = radius;
    }

    void SetBoxHalfExtents(const Vector3& halfExtents)
    {
        assert(type == BOX && "Collider is not a box!");
        box.halfExtents = halfExtents;
    }

    void SetCapsuleRadius(float radius)
    {
        assert(type == CAPSULE && "Collider is not a capsule!");
        capsule.radius = radius;
    }    
    void SetCapsuleHeight(float height)
    {
        assert(type == CAPSULE && "Collider is not a capsule!");
        capsule.height = height;
    }

#else // Release
    float GetSphereRadius() const { return sphere.radius; }
    const Vector3& GetBoxHalfExtents() const { return box.halfExtents; }
    float GetCapsuleRadius() const { return capsule.rdius; }
    float GetCapsuleHeight() const { return capsule.height; }
    void SetSphereRadius(float radius) { sphere.radius = radius; }
    void SetBoxHalfExtents(const Vector3& halfExtents) { box.halfExtents = halfExtents; }
    void SetCapsuleRadius(float radius) { capsule.radius = radius; }
    void SetCapsuleHeight(float height) { capsule.height = height; }
#endif
};
}// End namespace

#endif
