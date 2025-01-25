#include "MIRARigidBody.h"

using namespace MIRA;

RigidBody::RigidBody(float mass, float restitution) : mass(mass), restitution(restitution) {}

void RigidBody::ApplyForce(const Vector3& f) { if (mass > 0.0f) force = force + f; }

void RigidBody::ClearForces() { force = Vector3(0.0f, 0.0f, 0.0f); }

void RigidBody::Update(float deltaTime)
{
    if (mass > 0.0f)
    {
        Vector3 acceleration = force / mass;
        velocity = velocity + acceleration * deltaTime;
        position = position + velocity * deltaTime;
    }
    ClearForces(); // Always clear forces
}

float RigidBody::InvMass() { return (mass == 0.0f) ? 0.0f : 1.0f / mass; }
