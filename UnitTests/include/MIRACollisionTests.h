#include "MIRATestFramework.h"

#include "MIRACollider.h"
#include "MIRARigidBody.h"
#include "CollisionDetection.h"

using namespace MIRA;

#pragma region rigidbody
TEST_CASE(RigidBody_ApplyForce)
{
    RigidBody body(2.0f); // Mass = 2.0
    body.ApplyForce(Vector3(10.0f, 0.0f, 0.0f)); // Force = (10, 0, 0)

    ASSERT_EQUAL(body.force.x, 10.0f);
    ASSERT_EQUAL(body.force.y, 0.0f);
    ASSERT_EQUAL(body.force.z, 0.0f);
}

TEST_CASE(RigidBody_Integration)
{
    RigidBody body(1.0f); // Mass = 1.0
    body.ApplyForce(Vector3(2.0f, 0.0f, 0.0f));
    body.velocity = Vector3(1.0f, 0.0f, 0.0f);

    // Simulate 1 second with Euler integration
    body.velocity = body.velocity + body.force / body.mass * 1.0f;
    body.position = body.position + body.velocity * 1.0f;
    body.ClearForces();

    ASSERT_EQUAL(body.position.x, 1.0f + 2.0f); // Initial velocity (1) + force (2)
    ASSERT_EQUAL(body.force.x, 0.0f); // Forces should be cleared
}

TEST_CASE(RigidBody_ZeroMass_NoMovement)
{
    RigidBody body(0.0f); // Infinite mass
    body.ApplyForce(Vector3(100.0f, 0.0f, 0.0f));
    body.velocity = Vector3(1.0f, 0.0f, 0.0f);
        
    // Simulate 1 second
    body.Update(1.0f);

    ASSERT_EQUAL(body.velocity.x, 1.0f); // Velocity unchanged (infinite mass)
    ASSERT_EQUAL(body.position.x, 0.0f); // Position unchanged
}

TEST_CASE(RigidBody_NegativeVelocity)
{
    RigidBody body(1.0f);
    body.velocity = Vector3(-5.0f, 0.0f, 0.0f);
    body.position = Vector3(10.0f, 0.0f, 0.0f);

    // Simulate 1 second
    body.position = body.position + body.velocity * 1.0f;

    ASSERT_EQUAL(body.position.x, 5.0f); // 10 - 5 = 5
}
#pragma endregion

#pragma region collision
TEST_CASE(SphereCollider_AABB)
{
    RigidBody body;
    body.position = Vector3(1.0f, 2.0f, 3.0f);

    Collider sphereCollider(Collider::SPHERE, &body);
    sphereCollider.SetSphereRadius(2.0f);
    //SphereCollider sphere(&body, 2.0f); // Radius = 2.0

    AABB aabb = ComputeAABB(sphereCollider);

    // Test all components of the AABB
    ASSERT_FLOAT_EQUAL(aabb.min.x, 1.0f - 2.0f); // -1.0f
    ASSERT_FLOAT_EQUAL(aabb.min.y, 2.0f - 2.0f); // 0.0f
    ASSERT_FLOAT_EQUAL(aabb.min.z, 3.0f - 2.0f); // 1.0f
           
    ASSERT_FLOAT_EQUAL(aabb.max.x, 1.0f + 2.0f); // 3.0f
    ASSERT_FLOAT_EQUAL(aabb.max.y, 2.0f + 2.0f); // 4.0f (failure point)
    ASSERT_FLOAT_EQUAL(aabb.max.z, 3.0f + 2.0f); // 5.0f
}

TEST_CASE(BoxCollider_AABB)
{
    RigidBody body;
    body.position = Vector3(0.0f, 0.0f, 0.0f);
    Collider boxCollider(Collider::BOX, &body);
    boxCollider.SetBoxHalfExtents(Vector3(1.0f, 2.0f, 3.0f));

    AABB aabb = ComputeAABB(boxCollider);
    ASSERT_EQUAL(aabb.min.x, -1.0f);
    ASSERT_EQUAL(aabb.max.z, 3.0f);
}

TEST_CASE(BoxCollider_NonUniformHalfExtents)
{
    RigidBody body;
    body.position = Vector3(2.0f, 3.0f, 4.0f);
    Collider boxCollider(Collider::BOX, &body);
    boxCollider.SetBoxHalfExtents(Vector3(1.0f, 2.0f, 3.0f));

    AABB aabb = ComputeAABB(boxCollider);
    ASSERT_EQUAL(aabb.min.x, 1.0f); // 2 - 1 = 1
    ASSERT_EQUAL(aabb.max.y, 5.0f); // 3 + 2 = 5
    ASSERT_EQUAL(aabb.min.z, 1.0f); // 4 - 3 = 1
}

TEST_CASE(SphereCollider_NegativeRadius)
{
    RigidBody body;
    Collider sphereCollider(Collider::SPHERE, &body);
    sphereCollider.SetSphereRadius(-1.0f);

    AABB aabb = ComputeAABB(sphereCollider);
    ASSERT_TRUE(aabb.min.x > aabb.max.x); // AABB is invalid (test should fail)
}

TEST_CASE(SphereSphere_Collision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(3.0f, 0.0f, 0.0f);

    Collider sphereCollider1(Collider::SPHERE, &body1);
    sphereCollider1.SetSphereRadius(2.0f);
    Collider sphereCollider2(Collider::SPHERE, &body2);
    sphereCollider2.SetSphereRadius(2.0f);

    CollisionInfo info;
    bool isColliding = SphereSphereCollision(sphereCollider1, sphereCollider2, info);

    ASSERT_TRUE(isColliding);
    ASSERT_NEAR(info.depth, 1.0f, 0.0001f); // Distance = 3.0, sum of radii = 4.0 -> depth = 1.0
    ASSERT_EQUAL(info.normal.x, 1.0f); // Collision normal points from sphere1 to sphere2
}

TEST_CASE(SphereSphere_NoCollision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(5.0f, 0.0f, 0.0f);

    Collider sphereCollider1(Collider::SPHERE, &body1);
    sphereCollider1.SetSphereRadius(2.0f);
    Collider sphereCollider2(Collider::SPHERE, &body2);
    sphereCollider2.SetSphereRadius(2.0f);

    CollisionInfo info;
    bool isColliding = SphereSphereCollision(sphereCollider1, sphereCollider2, info);

    ASSERT_FALSE(isColliding);
}

TEST_CASE(SphereSphere_LargeVelocity)
{
    RigidBody body1, body2;
    body1.velocity = Vector3(1000.0f, 0.0f, 0.0f);
    body2.velocity = Vector3(-1000.0f, 0.0f, 0.0f);

    Collider sphereCollider1(Collider::SPHERE, &body1);
    sphereCollider1.SetSphereRadius(1.0f);
    Collider sphereCollider2(Collider::SPHERE, &body2);
    sphereCollider2.SetSphereRadius(1.0f);

    CollisionInfo info;
    bool isColliding = SphereSphereCollision(sphereCollider1, sphereCollider2, info);

    ASSERT_TRUE(isColliding); // Ensure no numerical instability
}

TEST_CASE(SphereSphere_NoCollision_MovingApart)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(3.0f, 0.0f, 0.0f);
    body1.velocity = Vector3(-1.0f, 0.0f, 0.0f);
    body2.velocity = Vector3(1.0f, 0.0f, 0.0f);

    Collider sphereCollider1(Collider::SPHERE, &body1);
    sphereCollider1.SetSphereRadius(1.0f);
    Collider sphereCollider2(Collider::SPHERE, &body2);
    sphereCollider2.SetSphereRadius(1.0f);

    CollisionInfo info;
    bool isColliding = SphereSphereCollision(sphereCollider1, sphereCollider2, info);

    ASSERT_FALSE(isColliding); // Objects moving away should not collide
}

TEST_CASE(SphereBox_Collision_Outside)
{
    RigidBody sphereBody, boxBody;
    sphereBody.position = Vector3(3.0f, 0.0f, 0.0f);
    boxBody.position = Vector3(0.0f, 0.0f, 0.0f);

    Collider sphereCollider(Collider::SPHERE, &sphereBody);
    sphereCollider.SetSphereRadius(2.0f);
    Collider boxCollider(Collider::BOX, &boxBody);
    boxCollider.SetBoxHalfExtents(Vector3(2.0f, 2.0f, 2.0f));

    CollisionInfo info;
    bool isColliding = SphereBoxCollision(sphereCollider, boxCollider, info);

    ASSERT_TRUE(isColliding);
    ASSERT_NEAR(info.depth, 1.0f, 0.0001f); // Distance from box edge (2) to sphere surface (3 - 2)
    ASSERT_EQUAL(info.normal.x, 1.0f); // Collision normal points from box to sphere
}

TEST_CASE(SphereBox_Collision_Inside)
{
    RigidBody sphereBody, boxBody;
    sphereBody.position = Vector3(0.0f, 0.0f, 0.0f);
    boxBody.position = Vector3(0.0f, 0.0f, 0.0f);

    Collider sphereCollider(Collider::SPHERE, &sphereBody);
    sphereCollider.SetSphereRadius(1.0f);
    Collider boxCollider(Collider::BOX, &boxBody);
    boxCollider.SetBoxHalfExtents(Vector3(2.0f, 2.0f, 2.0f)); // Box encloses sphere

    CollisionInfo info;
    bool isColliding = SphereBoxCollision(sphereCollider, boxCollider, info);

    ASSERT_TRUE(isColliding);
    ASSERT_NEAR(info.depth, 3.0f, 0.0001f); // Radius (1) + distance to nearest face (2)
    ASSERT_TRUE(info.normal.x != 0.0f || info.normal.y != 0.0f || info.normal.z != 0.0f);
}

TEST_CASE(SphereBox_Collision_EdgeTouching)
{
    RigidBody sphereBody, boxBody;
    sphereBody.position = Vector3(3.0f, 0.0f, 0.0f);
    boxBody.position = Vector3(0.0f, 0.0f, 0.0f);

    Collider sphereCollider(Collider::SPHERE, &sphereBody);
    sphereCollider.SetSphereRadius(1.0f);
    Collider boxCollider(Collider::BOX, &boxBody);
    boxCollider.SetBoxHalfExtents(Vector3(2.0f, 2.0f, 2.0f));

    CollisionInfo info;
    bool isColliding = SphereBoxCollision(sphereCollider, boxCollider, info);

    ASSERT_TRUE(isColliding);
    ASSERT_NEAR(info.depth, 0.0f, 0.0001f); // Just touching
}

TEST_CASE(SphereBox_NoCollision)
{
    RigidBody sphereBody, boxBody;
    sphereBody.position = Vector3(5.0f, 0.0f, 0.0f);
    boxBody.position = Vector3(0.0f, 0.0f, 0.0f);

    Collider sphereCollider(Collider::SPHERE, &sphereBody);
    sphereCollider.SetSphereRadius(1.0f);
    Collider boxCollider(Collider::BOX, &boxBody);
    boxCollider.SetBoxHalfExtents(Vector3(2.0f, 2.0f, 2.0f));

    CollisionInfo info;
    bool isColliding = SphereBoxCollision(sphereCollider, boxCollider, info);

    ASSERT_FALSE(isColliding);
}

TEST_CASE(BoxBox_Collision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(1.5f, 0.0f, 0.0f);

    Collider boxCollider1(Collider::BOX, &body1);
    boxCollider1.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));
    Collider boxCollider2(Collider::BOX, &body2);
    boxCollider2.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));

    CollisionInfo info;
    bool isColliding = BoxBoxCollision(boxCollider1, boxCollider2, info);

    ASSERT_TRUE(isColliding);
    ASSERT_NEAR(info.depth, 0.5f, 0.0001f); // Overlap = (1 + 1) - 1.5 = 0.5
    ASSERT_EQUAL(info.normal.x, 1.0f); // Normal points from box1 to box2
}

TEST_CASE(BoxBox_Collision_Edge)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(2.0f, 0.0f, 0.0f);

    Collider boxCollider1(Collider::BOX, &body1);
    boxCollider1.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));
    Collider boxCollider2(Collider::BOX, &body2);
    boxCollider2.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));

    CollisionInfo info;
    bool isColliding = BoxBoxCollision(boxCollider1, boxCollider2, info);

    ASSERT_TRUE(isColliding); // Edges barely touching
    ASSERT_NEAR(info.depth, 0.0f, 0.0001f); // Adjust based on your implementation
}

TEST_CASE(ResolveCollision_Impulse)
{
    RigidBody body1(1.0f, 0.5f); // Mass = 1.0, restitution = 0.5
    RigidBody body2(1.0f, 0.5f);
    body1.velocity = Vector3(2.0f, 0.0f, 0.0f);
    body2.velocity = Vector3(-1.0f, 0.0f, 0.0f);

    CollisionInfo info;
    info.normal = Vector3(1.0f, 0.0f, 0.0f); // Collision along X-axis
    info.depth = 0.5f;

    ResolveCollision(body1, body2, info);

    // Expected velocities after collision:
    // Relative velocity = (-1 - 2) = -3 -> impulse j = -(1 + 0.5) * (-3) / (1 + 1) = 2.25
    // body1.vel = 2 - (2.25 * 1) = -0.25
    // body2.vel = -1 + (2.25 * 1) = 1.25
    ASSERT_NEAR(body1.velocity.x, -0.25f, 0.0001f);
    ASSERT_NEAR(body2.velocity.x, 1.25f, 0.0001f);
}

TEST_CASE(ResolveCollision_PositionCorrection)
{
    RigidBody body1(1.0f);
    RigidBody body2(1.0f);
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(1.5f, 0.0f, 0.0f); // Overlapping by 0.5 units

    CollisionInfo info;
    info.normal = Vector3(1.0f, 0.0f, 0.0f);
    info.depth = 0.5f;

    ResolveCollision(body1, body2, info);

    // Position correction:
    // correction = (0.5 - 0.01) * 0.2 / (1 + 1) ~= 0.049
    // body1.pos -= 0.049 / 1 -> ~-0.049
    // body2.pos += 0.049 / 1 -> ~1.5 + 0.049 = 1.549
    ASSERT_NEAR(body1.position.x, -0.049f, 0.001f);
    ASSERT_NEAR(body2.position.x, 1.549f, 0.001f);
}

TEST_CASE(ResolveCollision_ZeroMass)
{
    RigidBody body1(0.0f, 0.5f); // Static object (mass = 0)
    RigidBody body2(1.0f, 0.5f); // Dynamic object (mass = 1.0)
    body2.velocity = Vector3(-2.0f, 0.0f, 0.0f);

    CollisionInfo info;
    info.normal = Vector3(1.0f, 0.0f, 0.0f); // Collision along X-axis
    info.depth = 0.5f;

    ResolveCollision(body1, body2, info);

    // Static object shouldn’t move
    ASSERT_EQUAL(body1.velocity.x, 0.0f);

    // Correct expected velocity for body2:
    // Initial velocity: -2.0f
    // Impulse j = (1 + 0.5) * 2.0f (velAlongNormal = -2.0f)
    // Velocity change: j / 1.0f = 3.0f
    // Final velocity: -2.0f + 3.0f = 1.0f
    ASSERT_NEAR(body2.velocity.x, 1.0f, 0.0001f);
}

TEST_CASE(ResolveCollision_HighRestitution)
{
    RigidBody body1(1.0f, 1.0f); // Restitution = 1.0 (perfect bounce)
    RigidBody body2(1.0f, 1.0f);
    body1.velocity = Vector3(3.0f, 0.0f, 0.0f);
    body2.velocity = Vector3(-1.0f, 0.0f, 0.0f);

    CollisionInfo info;
    info.normal = Vector3(1.0f, 0.0f, 0.0f);
    ResolveCollision(body1, body2, info);

    ASSERT_NEAR(body1.velocity.x, -1.0f, 0.0001f); // Swap velocities
    ASSERT_NEAR(body2.velocity.x, 3.0f, 0.0001f);
}

TEST_CASE(ResolveCollision_RestingContact)
{
    RigidBody body1(1.0f, 0.0f); // Restitution = 0 (no bounce)
    RigidBody body2(1.0f, 0.0f);
    body1.velocity = Vector3(1.0f, 0.0f, 0.0f);
    body2.velocity = Vector3(-1.0f, 0.0f, 0.0f);

    CollisionInfo info;
    info.normal = Vector3(1.0f, 0.0f, 0.0f);
    ResolveCollision(body1, body2, info);

    ASSERT_NEAR(body1.velocity.x, 0.0f, 0.0001f); // Velocities zeroed
    ASSERT_NEAR(body2.velocity.x, 0.0f, 0.0001f);
}
#pragma endregion