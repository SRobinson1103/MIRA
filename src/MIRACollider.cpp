#include "MIRACollider.h"

using namespace MIRA;

Collider::Collider(Type type, RigidBody* body) : type(type), body(body)
{
    if (type == SPHERE)
    {
        sphere.radius = 0.0f;
    }
    else if (type == BOX)
    {
        box.halfExtents = Vector3(0.0f);
    }
    else //capsule
    {
        capsule.height = 0.0f;
        capsule.radius = 0.0f;
    }
}
