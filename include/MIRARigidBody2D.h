#ifndef MIRA_RIGIDBODY2D_H
#define MIRA_RIGIDBODY2D_H

#include "MIRAVector.h"

namespace MIRA
{
class RigidBody2D
{
public:
    Vector2 position;
    Vector2 velocity;
    Vector2 force;
    float mass;
    float restitution; // Bounciness, 0.0 = no bounce, 1.0 = perfect bounce

    RigidBody2D(float mass = 1.0f, float restitution = 0.5f);

    void ApplyForce(const Vector2& f);

    void ClearForces();

    void Update(float deltaTime);

    float InvMass();
};
}// End namespace

#endif