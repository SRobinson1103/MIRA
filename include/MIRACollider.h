#ifndef MIRA_COLLIDER_H
#define MIRA_COLLIDER_H

#include "MIRARigidBody.h"

#include <cassert>

namespace MIRA
{
// avoids polymorphism for performance improvements
class Collider
{
public:
    enum Type { SPHERE, BOX };
    Type type;
    RigidBody* body;

    Collider(Type type, RigidBody* body);

private:
    union
    {
        struct { float radius; } sphere;
        struct { Vector3 halfExtents; } box;
    };

public:
    // Debug-only accessors and setters with validation
#if !defined(NDEBUG) // Active in debug builds
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
#else // Release: no checks, direct access and assigmnent
    float GetSphereRadius() const { return sphere.radius; }
    const Vector3& GetBoxHalfExtents() const { return box.halfExtents; }
    void SetSphereRadius(float radius) { sphere.radius = radius; }
    void SetBoxHalfExtents(const Vector3& halfExtents) { box.halfExtents = halfExtents; }
#endif
};
/*
class Collider
{
public:
    enum class Type { SPHERE, BOX };
    Type type;
    RigidBody* body; // Associated rigid body

    Collider(Type type, RigidBody* body);
    virtual ~Collider() = default;
};

class SphereCollider : public Collider
{
public:
    float radius;

    SphereCollider(RigidBody* body, float radius);
};

class BoxCollider : public Collider
{
public:
    Vector3 halfExtents; // Half the size of the box (e.g., (1,1,1) for a 2x2x2 box)

    BoxCollider(RigidBody* body, const Vector3& halfExtents);
};*/
}

#endif
