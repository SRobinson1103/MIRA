#include "MIRATestFramework.h"
#include "MIRAMatrix.h"
#include "MIRAVector.h"

using namespace MIRA;

#pragma region vector
TEST_CASE(Vector3_Addition)
{
    Vector3 v1(1.0f, 2.0f, 3.0f);
    Vector3 v2(4.0f, 5.0f, 6.0f);
    Vector3 result = v1 + v2;

    ASSERT_EQUAL(result.x, 5.0f);
    ASSERT_EQUAL(result.y, 7.0f);
    ASSERT_EQUAL(result.z, 9.0f);
}

TEST_CASE(Vector3_Subtraction)
{
    Vector3 v1(4.0f, 5.0f, 6.0f);
    Vector3 v2(1.0f, 2.0f, 3.0f);
    Vector3 result = v1 - v2;

    ASSERT_EQUAL(result.x, 3.0f);
    ASSERT_EQUAL(result.y, 3.0f);
    ASSERT_EQUAL(result.z, 3.0f);
}

TEST_CASE(Vector3_DotProduct)
{
    Vector3 v1(1.0f, 2.0f, 3.0f);
    Vector3 v2(4.0f, 5.0f, 6.0f);
    float result = v1.Dot(v2);

    ASSERT_EQUAL(result, 32.0f); // 1*4 + 2*5 + 3*6 = 32
}

TEST_CASE(Vector3_CrossProduct)
{
    Vector3 v1(1.0f, 0.0f, 0.0f);
    Vector3 v2(0.0f, 1.0f, 0.0f);
    Vector3 result = v1.Cross(v2);

    ASSERT_EQUAL(result.x, 0.0f);
    ASSERT_EQUAL(result.y, 0.0f);
    ASSERT_EQUAL(result.z, 1.0f); // Cross product of X and Y is Z
}

TEST_CASE(Vector3_Magnitude)
{
    Vector3 v(3.0f, 4.0f, 0.0f);
    float result = v.Magnitude();

    ASSERT_EQUAL(result, 5.0f); // sqrt(3^2 + 4^2) = 5
}

TEST_CASE(Vector3_Normalization)
{
    Vector3 v(3.0f, 4.0f, 0.0f);
    Vector3 result = v.Normalized();

    ASSERT_NEAR(result.x, 0.6f, 0.0001f); // 3/5 = 0.6
    ASSERT_NEAR(result.y, 0.8f, 0.0001f); // 4/5 = 0.8
    ASSERT_NEAR(result.z, 0.0f, 0.0001f); // 0/5 = 0
}
#pragma endregion

#pragma region matrix
TEST_CASE(Matrix4_Multiplication)
{
    Matrix4 m1;
    m1.m[0][0] = 1.0f; m1.m[0][1] = 2.0f; m1.m[0][2] = 3.0f; m1.m[0][3] = 4.0f;
    m1.m[1][0] = 5.0f; m1.m[1][1] = 6.0f; m1.m[1][2] = 7.0f; m1.m[1][3] = 8.0f;
    m1.m[2][0] = 9.0f; m1.m[2][1] = 10.0f; m1.m[2][2] = 11.0f; m1.m[2][3] = 12.0f;
    m1.m[3][0] = 13.0f; m1.m[3][1] = 14.0f; m1.m[3][2] = 15.0f; m1.m[3][3] = 16.0f;

    Matrix4 m2;
    m2.m[0][0] = 17.0f; m2.m[0][1] = 18.0f; m2.m[0][2] = 19.0f; m2.m[0][3] = 20.0f;
    m2.m[1][0] = 21.0f; m2.m[1][1] = 22.0f; m2.m[1][2] = 23.0f; m2.m[1][3] = 24.0f;
    m2.m[2][0] = 25.0f; m2.m[2][1] = 26.0f; m2.m[2][2] = 27.0f; m2.m[2][3] = 28.0f;
    m2.m[3][0] = 29.0f; m2.m[3][1] = 30.0f; m2.m[3][2] = 31.0f; m2.m[3][3] = 32.0f;

    Matrix4 result = m1 * m2;

    // Test all elements of the resulting matrix
    ASSERT_EQUAL(result.m[0][0], 250.0f);
    ASSERT_EQUAL(result.m[0][1], 260.0f);
    ASSERT_EQUAL(result.m[0][2], 270.0f);
    ASSERT_EQUAL(result.m[0][3], 280.0f);

    ASSERT_EQUAL(result.m[1][0], 618.0f);
    ASSERT_EQUAL(result.m[1][1], 644.0f);
    ASSERT_EQUAL(result.m[1][2], 670.0f);
    ASSERT_EQUAL(result.m[1][3], 696.0f);

    ASSERT_EQUAL(result.m[2][0], 986.0f);
    ASSERT_EQUAL(result.m[2][1], 1028.0f);
    ASSERT_EQUAL(result.m[2][2], 1070.0f);
    ASSERT_EQUAL(result.m[2][3], 1112.0f);

    ASSERT_EQUAL(result.m[3][0], 1354.0f);
    ASSERT_EQUAL(result.m[3][1], 1412.0f);
    ASSERT_EQUAL(result.m[3][2], 1470.0f);
    ASSERT_EQUAL(result.m[3][3], 1528.0f);
}

TEST_CASE(Matrix4_Translation)
{
    Matrix4 translation = Matrix4::Translate(Vector3(1.0f, 2.0f, 3.0f));

    ASSERT_EQUAL(translation.m[0][3], 1.0f); // Translation in X
    ASSERT_EQUAL(translation.m[1][3], 2.0f); // Translation in Y
    ASSERT_EQUAL(translation.m[2][3], 3.0f); // Translation in Z
}

TEST_CASE(Matrix4_RotationZ)
{
    float angle = 3.14159f / 2.0f; // 90 degrees
    Matrix4 rotation = Matrix4::RotateZ(angle);

    ASSERT_NEAR(rotation.m[0][0], 0.0f, 0.0001f); // cos(90°) = 0
    ASSERT_NEAR(rotation.m[0][1], -1.0f, 0.0001f); // -sin(90°) = -1
    ASSERT_NEAR(rotation.m[1][0], 1.0f, 0.0001f); // sin(90°) = 1
    ASSERT_NEAR(rotation.m[1][1], 0.0f, 0.0001f); // cos(90°) = 0
}

TEST_CASE(Matrix4_Scaling)
{
    Matrix4 scale = Matrix4::Scale(Vector3(2.0f, 3.0f, 4.0f));

    ASSERT_EQUAL(scale.m[0][0], 2.0f); // Scale in X
    ASSERT_EQUAL(scale.m[1][1], 3.0f); // Scale in Y
    ASSERT_EQUAL(scale.m[2][2], 4.0f); // Scale in Z
}
#pragma endregion
