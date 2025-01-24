#include "MIRACollider.h"

using namespace MIRA;

Collider::Collider(Type type, RigidBody* body) : type(type), body(body)
{
    if (type == SPHERE)
    {
        sphere.radius = 0.0f;
    }
    else
    {
        box.halfExtents = Vector3(0.0f);
    }
}

//SphereCollider::SphereCollider(RigidBody* body, float radius) : Collider(Type::SPHERE, body), radius(radius) {}

//BoxCollider::BoxCollider(RigidBody* body, const Vector3& halfExtents) : Collider(Type::BOX, body), halfExtents(halfExtents) {}
