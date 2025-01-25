#include "MIRARigidBody2D.h"

using namespace MIRA;

RigidBody2D::RigidBody2D(float mass, float restitution) : mass(mass), restitution(restitution) {}

void RigidBody2D::ApplyForce(const Vector2& f) { if (mass > 0.0f) force = force + f; }

void RigidBody2D::ClearForces() { force = Vector2(0.0f, 0.0f); }

void RigidBody2D::Update(float deltaTime)
{
    if (mass > 0.0f)
    {
        Vector2 acceleration = force / mass;
        velocity = velocity + acceleration * deltaTime;
        position = position + velocity * deltaTime;
    }
    ClearForces(); // Always clear forces
}

float RigidBody2D::InvMass() { return (mass == 0.0f) ? 0.0f : 1.0f / mass; }