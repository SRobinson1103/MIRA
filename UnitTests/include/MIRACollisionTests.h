#include "MIRATestFramework.h"

#include "CollisionDetection.h"
#include "CollisionDetection2D.h"

using namespace MIRA;

#pragma region AABB2D
TEST_CASE(ComputeAABB2D_Circle)
{
    RigidBody2D body;
    Collider2D circle(&body, Collider2D::Type2D::CIRCLE);
    circle.SetCircleRadius(2.0f);

    AABB2D aabb = ComputeAABB2D(circle);
    ASSERT_EQUAL(aabb.min_x, -2.0f);
    ASSERT_EQUAL(aabb.min_y, -2.0f);
    ASSERT_EQUAL(aabb.max_x, 2.0f);
    ASSERT_EQUAL(aabb.max_y, 2.0f);
}

TEST_CASE(ComputeAABB2D_Rectangle)
{
    RigidBody2D body;
    Collider2D rect(&body, Collider2D::Type2D::RECTANGLE);
    rect.SetRectLength(4.0f);
    rect.SetRectHeight(6.0f);

    AABB2D aabb = ComputeAABB2D(rect);
    ASSERT_EQUAL(aabb.min_x, -2.0f);
    ASSERT_EQUAL(aabb.min_y, -3.0f);
    ASSERT_EQUAL(aabb.max_x, 2.0f);
    ASSERT_EQUAL(aabb.max_y, 3.0f);
}

TEST_CASE(CheckAABBOverlap2D_Overlapping)
{
    AABB2D a = { -1.0f, 1.0f, -1.0f, 1.0f };
    AABB2D b = { -0.5f, 0.5f, -0.5f, 0.5f };
    ASSERT_EQUAL(CheckAABBOverlap2D(a, b), true);
}

TEST_CASE(CheckAABBOverlap2D_NonOverlapping)
{
    AABB2D a = { -1.0f, 1.0f, -1.0f, 1.0f };
    AABB2D b = { 2.0f, 2.0f, 3.0f, 3.0f };
    ASSERT_EQUAL(CheckAABBOverlap2D(a, b), false);
}
#pragma endregion

#pragma region circlecircle
TEST_CASE(CircleCircleCollision_Overlapping)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(1.5f, 0.0f);

    Collider2D circle1(&body1, Collider2D::Type2D::CIRCLE);
    Collider2D circle2(&body2, Collider2D::Type2D::CIRCLE);
    circle1.SetCircleRadius(1.0f);
    circle2.SetCircleRadius(1.0f);

    CollisionInfo2D info;
    bool result = CircleCircleCollision(circle1, circle2, info);

    ASSERT_EQUAL(result, true);
    ASSERT_NEAR(info.depth, 0.5f, 1e-6f);
    ASSERT_EQUAL(info.normal, Vector2(1.0f, 0.0f));
}

TEST_CASE(CircleCircleCollision_NonOverlapping)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(3.0f, 0.0f);

    Collider2D circle1(&body1, Collider2D::Type2D::CIRCLE);
    Collider2D circle2(&body2, Collider2D::Type2D::CIRCLE);
    circle1.SetCircleRadius(1.0f);
    circle2.SetCircleRadius(1.0f);

    CollisionInfo2D info;
    bool result = CircleCircleCollision(circle1, circle2, info);

    ASSERT_EQUAL(result, false);
}

TEST_CASE(CircleCircleCollision_ZeroDistance)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(0.0f, 0.0f);

    Collider2D circle1(&body1, Collider2D::Type2D::CIRCLE);
    Collider2D circle2(&body2, Collider2D::Type2D::CIRCLE);
    circle1.SetCircleRadius(1.0f);
    circle2.SetCircleRadius(1.0f);

    CollisionInfo2D info;
    bool result = CircleCircleCollision(circle1, circle2, info);

    ASSERT_EQUAL(result, true);
    ASSERT_NEAR(info.depth, 2.0f, 1e-6f);  // Sum of radii
    ASSERT_EQUAL(info.normal, Vector2(1.0f, 0.0f));  // Default direction
}
#pragma endregion

#pragma region circlerect
TEST_CASE(CircleRectCollision_Overlapping)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(1.5f, 0.0f);

    Collider2D circle(&body1, Collider2D::Type2D::CIRCLE);
    Collider2D rect(&body2, Collider2D::Type2D::RECTANGLE);
    circle.SetCircleRadius(1.0f);
    rect.SetRectLength(2.0f);
    rect.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = CircleRectCollision(circle, rect, info);

    ASSERT_EQUAL(result, true);
    ASSERT_NEAR(info.depth, 0.5f, 1e-6f);
    ASSERT_EQUAL(info.normal, Vector2(1.0f, 0.0f));
}

TEST_CASE(CircleRectCollision_NonOverlapping)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(3.0f, 0.0f);

    Collider2D circle(&body1, Collider2D::Type2D::CIRCLE);
    Collider2D rect(&body2, Collider2D::Type2D::RECTANGLE);
    circle.SetCircleRadius(1.0f);
    rect.SetRectLength(2.0f);
    rect.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = CircleRectCollision(circle, rect, info);

    ASSERT_EQUAL(result, false);
}

TEST_CASE(CircleRectCollision_TouchingEdge)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);  // Circle
    body2.position = Vector2(2.0f, 0.0f);  // Rectangle

    Collider2D circle(&body1, Collider2D::Type2D::CIRCLE);
    Collider2D rect(&body2, Collider2D::Type2D::RECTANGLE);
    circle.SetCircleRadius(1.0f);
    rect.SetRectLength(2.0f);
    rect.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = CircleRectCollision(circle, rect, info);

    ASSERT_EQUAL(result, true);
    ASSERT_NEAR(info.depth, 0.0f, 1e-6f);  // Just touching
    ASSERT_EQUAL(info.normal, Vector2(1.0f, 0.0f));
}

TEST_CASE(CircleRectCollision_CircleInsideRect)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);  // Circle
    body2.position = Vector2(0.0f, 0.0f);  // Rectangle

    Collider2D circle(&body1, Collider2D::Type2D::CIRCLE);
    Collider2D rect(&body2, Collider2D::Type2D::RECTANGLE);
    circle.SetCircleRadius(1.0f);
    rect.SetRectLength(4.0f);
    rect.SetRectHeight(4.0f);

    CollisionInfo2D info;
    bool result = CircleRectCollision(circle, rect, info);

    ASSERT_EQUAL(result, true);
    ASSERT_NEAR(info.depth, 1.0f, 1e-6f);  // Full radius penetration
    // Normal could point in any direction, but should be consistent
}

TEST_CASE(CircleRectCollision_CornerCase)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(3.0f, 3.0f);  // Circle
    body2.position = Vector2(2.0f, 2.0f);  // Rectangle (size 2x2)

    Collider2D circle(&body1, Collider2D::Type2D::CIRCLE);
    Collider2D rect(&body2, Collider2D::Type2D::RECTANGLE);
    circle.SetCircleRadius(1.0f);
    rect.SetRectLength(2.0f);
    rect.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = CircleRectCollision(circle, rect, info);

    ASSERT_EQUAL(result, true);
    // Normal should point diagonally
    Vector2 expectedNormal = (Vector2(3.0f, 3.0f) - Vector2(2.0f, 2.0f)).Normalized();
    ASSERT_NEAR(info.normal.x, expectedNormal.x, 1e-6f);
    ASSERT_NEAR(info.normal.y, expectedNormal.y, 1e-6f);
}
#pragma endregion

#pragma region rectrect
TEST_CASE(RectRectCollision_Overlapping)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(1.5f, 0.0f);

    Collider2D rect1(&body1, Collider2D::Type2D::RECTANGLE);
    Collider2D rect2(&body2, Collider2D::Type2D::RECTANGLE);
    rect1.SetRectLength(2.0f);
    rect1.SetRectHeight(2.0f);
    rect2.SetRectLength(2.0f);
    rect2.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = RectRectCollision(rect1, rect2, info);

    ASSERT_EQUAL(result, true);
    ASSERT_NEAR(info.depth, 0.5f, 1e-6f);
    ASSERT_EQUAL(info.normal, Vector2(1.0f, 0.0f));
}

TEST_CASE(RectRectCollision_NonOverlapping)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(3.0f, 0.0f);

    Collider2D rect1(&body1, Collider2D::Type2D::RECTANGLE);
    Collider2D rect2(&body2, Collider2D::Type2D::RECTANGLE);
    rect1.SetRectLength(2.0f);
    rect1.SetRectHeight(2.0f);
    rect2.SetRectLength(2.0f);
    rect2.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = RectRectCollision(rect1, rect2, info);

    ASSERT_EQUAL(result, false);
}

TEST_CASE(RectRectCollision_TouchingEdge)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(2.0f, 0.0f);  // Exactly touching

    Collider2D rect1(&body1, Collider2D::Type2D::RECTANGLE);
    Collider2D rect2(&body2, Collider2D::Type2D::RECTANGLE);
    rect1.SetRectLength(2.0f);
    rect1.SetRectHeight(2.0f);
    rect2.SetRectLength(2.0f);
    rect2.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = RectRectCollision(rect1, rect2, info);

    ASSERT_EQUAL(result, false);  // No penetration
}

// Test rectangle completely inside another rectangle
TEST_CASE(RectRectCollision_CompleteContainment)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);  // Larger rect
    body2.position = Vector2(0.0f, 0.0f);  // Smaller rect

    Collider2D rect1(&body1, Collider2D::Type2D::RECTANGLE);
    Collider2D rect2(&body2, Collider2D::Type2D::RECTANGLE);
    rect1.SetRectLength(4.0f);
    rect1.SetRectHeight(4.0f);
    rect2.SetRectLength(2.0f);
    rect2.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = RectRectCollision(rect1, rect2, info);

    ASSERT_EQUAL(result, true);
    ASSERT_NEAR(info.depth, 2.0f, 1e-6f);  // Half of smaller rect's size
    ASSERT_EQUAL(info.normal, Vector2(1.0f, 0.0f));  // Could be any axis
}

TEST_CASE(RectRectCollision_PartialOverlap)
{
    RigidBody2D body1, body2;
    body1.position = Vector2(0.0f, 0.0f);
    body2.position = Vector2(1.5f, 3.0f);  // Overlap on X but not Y

    Collider2D rect1(&body1, Collider2D::Type2D::RECTANGLE);
    Collider2D rect2(&body2, Collider2D::Type2D::RECTANGLE);
    rect1.SetRectLength(2.0f);
    rect1.SetRectHeight(2.0f);
    rect2.SetRectLength(2.0f);
    rect2.SetRectHeight(2.0f);

    CollisionInfo2D info;
    bool result = RectRectCollision(rect1, rect2, info);

    ASSERT_EQUAL(result, false);  // No Y-axis overlap
}

#pragma endregion

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
        
    body.Update(1.0f);

    ASSERT_EQUAL(body.velocity.x, 1.0f); // Velocity unchanged (infinite mass)
    ASSERT_EQUAL(body.position.x, 0.0f); // Position unchanged
}

TEST_CASE(RigidBody_NegativeVelocity)
{
    RigidBody body(1.0f);
    body.velocity = Vector3(-5.0f, 0.0f, 0.0f);
    body.position = Vector3(10.0f, 0.0f, 0.0f);

    body.position = body.position + body.velocity * 1.0f;

    ASSERT_EQUAL(body.position.x, 5.0f); // 10 - 5 = 5
}
#pragma endregion

#pragma region AABB
TEST_CASE(SphereCollider_AABB)
{
    RigidBody body;
    body.position = Vector3(1.0f, 2.0f, 3.0f);

    Collider sphereCollider(Collider::SPHERE, &body);
    sphereCollider.SetSphereRadius(2.0f);

    AABB aabb = ComputeAABB(sphereCollider);

    // Test all components of the AABB
    ASSERT_EQUAL(aabb.min.x, 1.0f - 2.0f); // -1.0f
    ASSERT_EQUAL(aabb.min.y, 2.0f - 2.0f); // 0.0f
    ASSERT_EQUAL(aabb.min.z, 3.0f - 2.0f); // 1.0f
           
    ASSERT_EQUAL(aabb.max.x, 1.0f + 2.0f); // 3.0f
    ASSERT_EQUAL(aabb.max.y, 2.0f + 2.0f); // 4.0f
    ASSERT_EQUAL(aabb.max.z, 3.0f + 2.0f); // 5.0f
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
    ASSERT_TRUE(aabb.min.x > aabb.max.x);
}

TEST_CASE(ComputeAABB_Capsule)
{
    RigidBody body;
    body.position = Vector3(0.0f, 0.0f, 0.0f);

    Collider capsuleCollider(Collider::CAPSULE, &body);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);

    AABB aabb = ComputeAABB(capsuleCollider);

    // Expected AABB for a vertically aligned capsule:
    // min = (-radius, -halfHeight - radius, -radius)
    // max = (radius, halfHeight + radius, radius)
    ASSERT_EQUAL(aabb.min.x, -1.0f);
    ASSERT_EQUAL(aabb.min.y, -2.0f);
    ASSERT_EQUAL(aabb.min.z, -1.0f);
    ASSERT_EQUAL(aabb.max.x, 1.0f);
    ASSERT_EQUAL(aabb.max.y, 2.0f);
    ASSERT_EQUAL(aabb.max.z, 1.0f);
}

TEST_CASE(ComputeAABB_Capsule_Offset)
{
    RigidBody body;
    body.position = Vector3(1.0f, 2.0f, 3.0f);

    Collider capsuleCollider(Collider::CAPSULE, &body);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);

    AABB aabb = ComputeAABB(capsuleCollider);

    // Expected AABB for a vertically aligned capsule at (1, 2, 3):
    // min = (1 - radius, 2 - halfHeight - radius, 3 - radius)
    // max = (1 + radius, 2 + halfHeight + radius, 3 + radius)
    ASSERT_EQUAL(aabb.min.x, 0.0f);
    ASSERT_EQUAL(aabb.min.y, 0.0f);
    ASSERT_EQUAL(aabb.min.z, 2.0f);
    ASSERT_EQUAL(aabb.max.x, 2.0f);
    ASSERT_EQUAL(aabb.max.y, 4.0f);
    ASSERT_EQUAL(aabb.max.z, 4.0f);
}
#pragma endregion

#pragma region spheresphere
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
    ASSERT_EQUAL(info.normal.x, 1.0f); // Collision normal points from sphere 1 to sphere 2
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
#pragma endregion

#pragma region spherebox
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
#pragma endregion

#pragma region boxbox
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
    ASSERT_EQUAL(info.normal.x, 1.0f); // Normal points from box 1 to box 2
}

TEST_CASE(BoxBoxCollision_NoCollision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(3.0f, 0.0f, 0.0f);

    Collider boxCollider1(Collider::BOX, &body1);
    boxCollider1.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));
    Collider boxCollider2(Collider::BOX, &body2);
    boxCollider2.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));

    CollisionInfo info;
    bool isColliding = BoxBoxCollision(boxCollider1, boxCollider2, info);

    ASSERT_FALSE(isColliding);
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
#pragma endregion

#pragma region capsulecapsule
TEST_CASE(CapsuleCapsuleCollision_NoCollision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(3.0f, 0.0f, 0.0f);

    Collider capsuleCollider1(Collider::CAPSULE, &body1);
    Collider capsuleCollider2(Collider::CAPSULE, &body2);
    capsuleCollider1.SetCapsuleRadius(1.0f);
    capsuleCollider1.SetCapsuleHeight(2.0f);
    capsuleCollider2.SetCapsuleRadius(1.0f);
    capsuleCollider2.SetCapsuleHeight(2.0f);

    CollisionInfo info;
    bool result = CapsuleCapsuleCollision(capsuleCollider1, capsuleCollider2, info);

    ASSERT_FALSE(result);
}

TEST_CASE(CapsuleCapsuleCollision_Collision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(1.5f, 0.0f, 0.0f);

    Collider capsuleCollider1(Collider::CAPSULE, &body1);
    Collider capsuleCollider2(Collider::CAPSULE, &body2);
    capsuleCollider1.SetCapsuleRadius(1.0f);
    capsuleCollider1.SetCapsuleHeight(2.0f);
    capsuleCollider2.SetCapsuleRadius(1.0f);
    capsuleCollider2.SetCapsuleHeight(2.0f);

    CollisionInfo info;
    bool result = CapsuleCapsuleCollision(capsuleCollider1, capsuleCollider2, info);

    ASSERT_TRUE(result);
    ASSERT_FLOAT_EQUAL(info.depth, 0.5f);
    ASSERT_EQUAL(info.normal.x, 1.0f);
    ASSERT_EQUAL(info.normal.y, 0.0f);
    ASSERT_EQUAL(info.normal.z, 0.0f);
}

TEST_CASE(CapsuleCapsuleCollision_Parallel)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f); // Capsule 1
    body2.position = Vector3(0.0f, 1.5f, 0.0f); // Capsule 2 (parallel)

    Collider capsule1(Collider::CAPSULE, &body1);
    Collider capsule2(Collider::CAPSULE, &body2);
    capsule1.SetCapsuleRadius(1.0f);
    capsule1.SetCapsuleHeight(2.0f);
    capsule2.SetCapsuleRadius(1.0f);
    capsule2.SetCapsuleHeight(2.0f);

    CollisionInfo info;
    bool result = CapsuleCapsuleCollision(capsule1, capsule2, info);

    ASSERT_TRUE(result);
    ASSERT_FLOAT_EQUAL(info.depth, 2.0f);
    ASSERT_EQUAL(info.normal.x, 0.0f);
    ASSERT_EQUAL(info.normal.y, 1.0f);
    ASSERT_EQUAL(info.normal.z, 0.0f);
}

TEST_CASE(CapsuleCapsuleCollision_Perpendicular)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f); // Capsule 1 (vertical)
    body2.position = Vector3(1.5f, 0.0f, 0.0f); // Capsule 2 (horizontal)

    Collider capsule1(Collider::CAPSULE, &body1);
    Collider capsule2(Collider::CAPSULE, &body2);
    capsule1.SetCapsuleRadius(1.0f);
    capsule1.SetCapsuleHeight(2.0f);
    capsule2.SetCapsuleRadius(1.0f);
    capsule2.SetCapsuleHeight(2.0f);

    CollisionInfo info;
    bool result = CapsuleCapsuleCollision(capsule1, capsule2, info);

    ASSERT_TRUE(result);
    ASSERT_FLOAT_EQUAL(info.depth, 0.5f);
    ASSERT_EQUAL(info.normal.x, 1.0f);
    ASSERT_EQUAL(info.normal.y, 0.0f);
    ASSERT_EQUAL(info.normal.z, 0.0f);
}
#pragma endregion

#pragma region capsulesphere
TEST_CASE(CapsuleSphereCollision_NoCollision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(3.0f, 0.0f, 0.0f);

    Collider capsuleCollider(Collider::CAPSULE, &body1);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);
    Collider sphereCollider(Collider::SPHERE, &body2);
    sphereCollider.SetSphereRadius(1.0f);

    CollisionInfo info;
    bool result = CapsuleSphereCollision(capsuleCollider, sphereCollider, info);

    ASSERT_FALSE(result);
}

TEST_CASE(CapsuleSphereCollision_Collision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(1.5f, 0.0f, 0.0f);

    Collider capsuleCollider(Collider::CAPSULE, &body1);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);
    Collider sphereCollider(Collider::SPHERE, &body2);
    sphereCollider.SetSphereRadius(1.0f);

    CollisionInfo info;
    bool result = CapsuleSphereCollision(capsuleCollider, sphereCollider, info);

    ASSERT_TRUE(result);
    ASSERT_FLOAT_EQUAL(info.depth, 0.5f);
    ASSERT_EQUAL(info.normal.x, 1.0f);
    ASSERT_EQUAL(info.normal.y, 0.0f);
    ASSERT_EQUAL(info.normal.z, 0.0f);
}

TEST_CASE(CapsuleSphereCollision_stacked)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(0.0f, 2.0f, 0.0f); // Sphere at capsule's top

    Collider capsuleCollider(Collider::CAPSULE, &body1);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);

    Collider sphereCollider(Collider::SPHERE, &body2);
    sphereCollider.SetSphereRadius(1.0f);

    CollisionInfo info;
    bool result = CapsuleSphereCollision(capsuleCollider, sphereCollider, info);

    ASSERT_TRUE(result);
    ASSERT_FLOAT_EQUAL(info.depth, 1.0f);
    ASSERT_EQUAL(info.normal.x, 0.0f);
    ASSERT_EQUAL(info.normal.y, 1.0f);
    ASSERT_EQUAL(info.normal.z, 0.0f);
}

TEST_CASE(CapsuleSphereCollision_NoOverlap)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(0.0f, 3.0f, 0.0f); // Sphere too far

    Collider capsuleCollider(Collider::CAPSULE, &body1);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);

    Collider sphereCollider(Collider::SPHERE, &body2);
    sphereCollider.SetSphereRadius(1.0f);

    CollisionInfo info;
    bool result = CapsuleSphereCollision(capsuleCollider, sphereCollider, info);

    ASSERT_FALSE(result);
}
#pragma endregion

#pragma region capsulebox
TEST_CASE(CapsuleBoxCollision_NoCollision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(3.0f, 0.0f, 0.0f);

    Collider capsuleCollider(Collider::CAPSULE, &body1);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);

    Collider boxCollider(Collider::BOX, &body2);
    boxCollider.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));

    CollisionInfo info;
    bool result = CapsuleBoxCollision(capsuleCollider, boxCollider, info);

    ASSERT_FALSE(result);
}

TEST_CASE(CapsuleBoxCollision_Collision)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(1.5f, 0.0f, 0.0f);

    Collider capsuleCollider(Collider::CAPSULE, &body1);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);

    Collider boxCollider(Collider::BOX, &body2);
    boxCollider.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));

    CollisionInfo info;
    bool result = CapsuleBoxCollision(capsuleCollider, boxCollider, info);

    ASSERT_TRUE(result);
    ASSERT_FLOAT_EQUAL(info.depth, 0.5f);
    ASSERT_EQUAL(info.normal.x, 1.0f);
    ASSERT_EQUAL(info.normal.y, 0.0f);
    ASSERT_EQUAL(info.normal.z, 0.0f);
}

TEST_CASE(CapsuleBoxCollision_stacked)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f);
    body2.position = Vector3(0.0f, 2.0f, 0.0f); // Box at capsule's top

    Collider capsuleCollider(Collider::CAPSULE, &body1);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);

    Collider boxCollider(Collider::BOX, &body2);
    boxCollider.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));

    CollisionInfo info;
    bool result = CapsuleBoxCollision(capsuleCollider, boxCollider, info);

    ASSERT_TRUE(result);
    ASSERT_FLOAT_EQUAL(info.depth, 1.0f);
    ASSERT_EQUAL(info.normal.x, 0.0f);
    ASSERT_EQUAL(info.normal.y, 1.0f);
    ASSERT_EQUAL(info.normal.z, 0.0f);
}

TEST_CASE(CapsuleBoxCollision_NoOverlap)
{
    RigidBody body1, body2;
    body1.position = Vector3(0.0f, 0.0f, 0.0f); // Capsule
    body2.position = Vector3(0.0f, 3.0f, 0.0f); // Box too far

    Collider capsuleCollider(Collider::CAPSULE, &body1);
    capsuleCollider.SetCapsuleRadius(1.0f);
    capsuleCollider.SetCapsuleHeight(2.0f);

    Collider boxCollider(Collider::BOX, &body2);
    boxCollider.SetBoxHalfExtents(Vector3(1.0f, 1.0f, 1.0f));

    CollisionInfo info;
    bool result = CapsuleBoxCollision(capsuleCollider, boxCollider, info);

    ASSERT_FALSE(result);
}
#pragma endregion

#pragma region resolveCollision
TEST_CASE(ResolveCollision_Impulse)
{
    RigidBody body1(1.0f, 0.5f);
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
    // body1.pos -= 0.049 / 1 -> ~ -0.049
    // body2.pos += 0.049 / 1 -> ~ 1.5 + 0.049 = 1.549
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