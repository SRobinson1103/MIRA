#ifndef MIRA_RIGID_BODY_H
#define MIRA_RIGID_BODY_H

#include "MIRAVector.h"

namespace MIRA
{
class RigidBody
{
public:
    Vector3 position;
    Vector3 velocity;
    Vector3 force;
    float mass;
    float restitution; // Bounciness, 0.0 = no bounce, 1.0 = perfect bounce

    RigidBody(float mass = 1.0f, float restitution = 0.5f);

    void ApplyForce(const Vector3& f);

    void ClearForces();

    void Update(float deltaTime);

    float InvMass();
};
}
#endif
