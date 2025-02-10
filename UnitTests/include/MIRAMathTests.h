#ifndef MIRA_MATH_TESTS_H
#define MIRA_MATH_TESTS_H

#include "MIRATestFramework.h"

#include "MIRAVector.h"
#include "MIRAMatrix.h"
#include "MIRAQuaternion.h"

using namespace MIRA;

#pragma region vector2
TEST_CASE(Vector2_DefaultConstructor)
{
    Vector2 v;
    ASSERT_EQUAL(v.x, 0.0f);
    ASSERT_EQUAL(v.y, 0.0f);
}

TEST_CASE(Vector2_ParameterizedConstructor)
{
    Vector2 v(3.0f, 4.0f);
    ASSERT_EQUAL(v.x, 3.0f);
    ASSERT_EQUAL(v.y, 4.0f);
}

TEST_CASE(Vector2_Addition)
{
    Vector2 v1(3.0f, 4.0f);
    Vector2 v2(1.0f, 2.0f);
    Vector2 result = v1 + v2;
    ASSERT_EQUAL(result.x, 4.0f);
    ASSERT_EQUAL(result.y, 6.0f);
}

TEST_CASE(Vector2_Subtraction)
    {
    Vector2 v1(3.0f, 4.0f);
    Vector2 v2(1.0f, 2.0f);
    Vector2 result = v1 - v2;
    ASSERT_EQUAL(result.x, 2.0f);
    ASSERT_EQUAL(result.y, 2.0f);
}

TEST_CASE(Vector2_ScalarMult)
{
    Vector2 v(3.0f, 4.0f);
    Vector2 result = v * 2.0f;
    ASSERT_EQUAL(result.x, 6.0f);
    ASSERT_EQUAL(result.y, 8.0f);
}

TEST_CASE(Vector2_ScalarDiv)
{
    Vector2 v(3.0f, 4.0f);
    Vector2 result = v / 2.0f;
    ASSERT_EQUAL(result.x, 1.5f);
    ASSERT_EQUAL(result.y, 2.0f);
}

TEST_CASE(Vector2_DotProduct)
{
    Vector2 v1(3.0f, 4.0f);
    Vector2 v2(1.0f, 2.0f);
    float dot = v1.Dot(v2);
    ASSERT_EQUAL(dot, 11.0f); // 3*1 + 4*2 = 11
}

TEST_CASE(Vector2_Magnitude)
{
    Vector2 v(3.0f, 4.0f);
    float mag = v.Magnitude();
    ASSERT_NEAR(mag, 5.0f, 1e-6f); // sqrt(3^2 + 4^2) = 5
}

TEST_CASE(Vector2_Normalization)
{
    Vector2 v(3.0f, 4.0f);
    Vector2 normalized = v.Normalized();
    ASSERT_NEAR(normalized.x, 0.6f, 1e-6f); // 3/5 = 0.6
    ASSERT_NEAR(normalized.y, 0.8f, 1e-6f); // 4/5 = 0.8
}

TEST_CASE(Vector2_NormalizeZeroVector)
{
    Vector2 zeroVec(0.0f, 0.0f);
    Vector2 zeroNormalized = zeroVec.Normalized();
    ASSERT_EQUAL(zeroNormalized.x, 0.0f);
    ASSERT_EQUAL(zeroNormalized.y, 0.0f);
}

TEST_CASE(Vector2_AbsoluteValue)
{
    Vector2 v(-3.0f, -4.0f);
    Vector2 absV = v.Abs();
    ASSERT_EQUAL(absV.x, 3.0f);
    ASSERT_EQUAL(absV.y, 4.0f);
}

TEST_CASE(Vector2_Equality)
{
    Vector2 v1(3.0f, 4.0f);
    Vector2 v2(3.0f, 4.0f);
    ASSERT_EQUAL(v1 == v2, true);
}

TEST_CASE(Vector2_Inequality)
{
    Vector2 v1(3.0f, 4.0f);
    Vector2 v2(3.0f, 5.0f);
    ASSERT_EQUAL(v1 != v2, true);
}
#pragma endregion

#pragma region vector3
TEST_CASE(Vector3_ScalarMult)
{
    float scalar = 2.0f;
    Vector3 v(1.0f, 2.0f, 3.0f);
    Vector3 result1 = v * scalar;
    Vector3 result2 = scalar * v;

    ASSERT_EQUAL(result1.x, 2.0f);
    ASSERT_EQUAL(result1.y, 4.0f);
    ASSERT_EQUAL(result1.z, 6.0f);

    ASSERT_EQUAL(result2.x, 2.0f);
    ASSERT_EQUAL(result2.y, 4.0f);
    ASSERT_EQUAL(result2.z, 6.0f);
}

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

TEST_CASE(Vector3_SubscriptOperator_ReadWrite)
{
    Vector3 vec(1.0f, 2.0f, 3.0f);

    // Read
    ASSERT_EQUAL(vec[0], 1.0f);
    ASSERT_EQUAL(vec[1], 2.0f);
    ASSERT_EQUAL(vec[2], 3.0f);

    // Write
    vec[0] = 10.0f;
    vec[1] = 20.0f;
    vec[2] = 30.0f;
    ASSERT_EQUAL(vec.x, 10.0f);
    ASSERT_EQUAL(vec.y, 20.0f);
    ASSERT_EQUAL(vec.z, 30.0f);
}

TEST_CASE(Vector3_SubscriptOperator_OutOfRange)
{
    Vector3 vec;
    bool exceptionThrown = false;
    try
    {
        float val = vec[3]; // Invalid index
    }
    catch (const std::out_of_range&)
    {
        exceptionThrown = true;
    }
    ASSERT_TRUE(exceptionThrown);
}

TEST_CASE(Vector3_SubscriptOperator_Stress)
{
    Vector3 vec;
    for (int i = 0; i < 3; ++i)
    {
        vec[i] = static_cast<float>(i);
    }
    ASSERT_EQUAL(vec.x, 0.0f);
    ASSERT_EQUAL(vec.y, 1.0f);
    ASSERT_EQUAL(vec.z, 2.0f);
}

TEST_CASE(Vector3_Abs_Positive)
{
    Vector3 vec(1.0f, 2.0f, 3.0f);
    Vector3 result = vec.Abs();
    ASSERT_EQUAL(result.x, 1.0f);
    ASSERT_EQUAL(result.y, 2.0f);
    ASSERT_EQUAL(result.z, 3.0f);
}

TEST_CASE(Vector3_Abs_Negative)
{
    Vector3 vec(-1.0f, -2.0f, -3.0f);
    Vector3 result = vec.Abs();
    ASSERT_EQUAL(result.x, 1.0f);
    ASSERT_EQUAL(result.y, 2.0f);
    ASSERT_EQUAL(result.z, 3.0f);
}

TEST_CASE(Vector3_Abs_Mixed)
{
    Vector3 vec(-5.0f, 0.0f, 3.0f);
    Vector3 result = vec.Abs();
    ASSERT_EQUAL(result.x, 5.0f);
    ASSERT_EQUAL(result.y, 0.0f);
    ASSERT_EQUAL(result.z, 3.0f);
}

TEST_CASE(Vector3_Abs_Zero)
{
    Vector3 vec(0.0f, -0.0f, 0.0f);
    Vector3 result = vec.Abs();
    ASSERT_EQUAL(result.x, 0.0f);
    ASSERT_EQUAL(result.y, 0.0f);
    ASSERT_EQUAL(result.z, 0.0f);
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

TEST_CASE(Vector3_CrossProduct_Orthogonal)
{
    Vector3 v1(1.0f, 0.0f, 0.0f);
    Vector3 v2(0.0f, 1.0f, 0.0f);
    Vector3 result = v1.Cross(v2);

    ASSERT_EQUAL(result.x, 0.0f);
    ASSERT_EQUAL(result.y, 0.0f);
    ASSERT_EQUAL(result.z, 1.0f); // Cross product of X and Y is Z
}

TEST_CASE(Vector3_Normalization_UnitVector)
{
    Vector3 v(3.0f, 4.0f, 0.0f);
    Vector3 result = v.Normalized();

    ASSERT_NEAR(result.x, 0.6f, 0.0001f); // 3/5 = 0.6
    ASSERT_NEAR(result.y, 0.8f, 0.0001f); // 4/5 = 0.8
    ASSERT_NEAR(result.z, 0.0f, 0.0001f); // 0/5 = 0
}

TEST_CASE(Vector3_Distance)
{
    Vector3 v1(1.0f, 2.0f, 3.0f);
    Vector3 v2(4.0f, 6.0f, 8.0f);
    float result = Vector3::Distance(v1, v2);

    ASSERT_NEAR(result, 7.0710678f, 0.0001f); // sqrt(3^2 + 4^2 + 5^2) = 7.0710678
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

TEST_CASE(Matrix4_Inverse)
{
    Matrix4 mat;
    mat.m[0][0] = 2.0f; mat.m[0][1] = 0.0f; mat.m[0][2] = 0.0f; mat.m[0][3] = 0.0f;
    mat.m[1][0] = 0.0f; mat.m[1][1] = 2.0f; mat.m[1][2] = 0.0f; mat.m[1][3] = 0.0f;
    mat.m[2][0] = 0.0f; mat.m[2][1] = 0.0f; mat.m[2][2] = 2.0f; mat.m[2][3] = 0.0f;
    mat.m[3][0] = 0.0f; mat.m[3][1] = 0.0f; mat.m[3][2] = 0.0f; mat.m[3][3] = 2.0f;

    Matrix4 inv = mat.Inverse();

    ASSERT_EQUAL(inv.m[0][0], 0.5f);
    ASSERT_EQUAL(inv.m[1][1], 0.5f);
    ASSERT_EQUAL(inv.m[2][2], 0.5f);
    ASSERT_EQUAL(inv.m[3][3], 0.5f);
}

TEST_CASE(Matrix4_Transpose)
{
    Matrix4 mat;
    mat.m[0][1] = 2.0f;
    mat.m[1][0] = 3.0f;
    Matrix4 result = mat.Transpose();

    ASSERT_EQUAL(result.m[0][1], 3.0f);
    ASSERT_EQUAL(result.m[1][0], 2.0f);
}

TEST_CASE(Matrix4_RotationX)
{
    float angle = 3.14159f / 2.0f; // 90 degrees
    Matrix4 result = Matrix4::RotateX(angle);

    ASSERT_NEAR(result.m[1][1], 0.0f, 0.0001f); // cos(90°) = 0
    ASSERT_NEAR(result.m[1][2], -1.0f, 0.0001f); // -sin(90°) = -1
    ASSERT_NEAR(result.m[2][1], 1.0f, 0.0001f); // sin(90°) = 1
    ASSERT_NEAR(result.m[2][2], 0.0f, 0.0001f); // cos(90°) = 0
}

TEST_CASE(Matrix4_Inverse_Identity)
{
    Matrix4 mat; // Identity matrix
    Matrix4 result = mat.Inverse();

    ASSERT_EQUAL(result.m[0][0], 1.0f);
    ASSERT_EQUAL(result.m[1][1], 1.0f);
    ASSERT_EQUAL(result.m[2][2], 1.0f);
    ASSERT_EQUAL(result.m[3][3], 1.0f);
}

TEST_CASE(Matrix4_Inverse_Transform)
{
    Matrix4 translate = Matrix4::Translate(Vector3(2.0f, 3.0f, 4.0f));
    Matrix4 invTranslate = translate.Inverse();
    Matrix4 result = translate * invTranslate;

    // The product should be the identity matrix
    for (int i = 0; i < 4; ++i) 
   {
        for (int j = 0; j < 4; ++j)
        {
            if (i == j)
            {
                ASSERT_NEAR(result.m[i][j], 1.0f, 0.0001f);
            }
            else
            {
                ASSERT_NEAR(result.m[i][j], 0.0f, 0.0001f);
            }
        }
    }
}

TEST_CASE(Matrix4_Inverse_Rotation)
{
    Matrix4 rot = Matrix4::RotateY(3.14159f / 2.0f); // 90-degree rotation
    Matrix4 invRot = rot.Inverse(); // Should be -90 degrees

    Vector3 original(1.0f, 0.0f, 0.0f);
    Vector3 rotated = rot * original; // Rotate to (0, 0, -1)
    Vector3 inverted = invRot * rotated; // Rotate back to (1, 0, 0)

    ASSERT_NEAR(inverted.x, 1.0f, 0.0001f);
    ASSERT_NEAR(inverted.y, 0.0f, 0.0001f);
    ASSERT_NEAR(inverted.z, 0.0f, 0.0001f);
}

TEST_CASE(Matrix4_Vector3_Multiplication)
{
    Matrix4 mat = Matrix4::Translate(Vector3(2.0f, 3.0f, 4.0f));
    Vector3 vec(1.0f, 1.0f, 1.0f);
    Vector3 result = mat * vec;

    ASSERT_NEAR(result.x, 3.0f, 0.0001f); // 1 + 2
    ASSERT_NEAR(result.y, 4.0f, 0.0001f); // 1 + 3
    ASSERT_NEAR(result.z, 5.0f, 0.0001f); // 1 + 4
}
#pragma endregion

#pragma region quaternion
TEST_CASE(Quaternion_Multiplication)
{
    Quaternion q1(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion q2(5.0f, 6.0f, 7.0f, 8.0f);
    Quaternion result = q1 * q2;

    ASSERT_EQUAL(result.w, -60.0f);
    ASSERT_EQUAL(result.x, 12.0f);
    ASSERT_EQUAL(result.y, 30.0f);
    ASSERT_EQUAL(result.z, 24.0f);
}

TEST_CASE(Quaternion_ToMatrix)
{
    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f); // Identity quaternion
    Matrix4 mat = q.ToMatrix();

    ASSERT_EQUAL(mat.m[0][0], 1.0f);
    ASSERT_EQUAL(mat.m[1][1], 1.0f);
    ASSERT_EQUAL(mat.m[2][2], 1.0f);
    ASSERT_EQUAL(mat.m[3][3], 1.0f);
}

TEST_CASE(Quaternion_Conjugate)
{
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion result = q.Conjugate();

    ASSERT_EQUAL(result.w, 1.0f);
    ASSERT_EQUAL(result.x, -2.0f);
    ASSERT_EQUAL(result.y, -3.0f);
    ASSERT_EQUAL(result.z, -4.0f);
}

TEST_CASE(Quaternion_Normalization)
{
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion result = q.Normalized();

    float norm = std::sqrt(1.0f + 4.0f + 9.0f + 16.0f); // sqrt(30)
    ASSERT_NEAR(result.w, 1.0f / norm, 0.0001f);
    ASSERT_NEAR(result.x, 2.0f / norm, 0.0001f);
    ASSERT_NEAR(result.y, 3.0f / norm, 0.0001f);
    ASSERT_NEAR(result.z, 4.0f / norm, 0.0001f);
}

TEST_CASE(Quaternion_Slerp)
{
    Quaternion q1(1.0f, 0.0f, 0.0f, 0.0f); // Identity
    Quaternion q2(0.0f, 1.0f, 0.0f, 0.0f); // 180° rotation around X-axis
    Quaternion result = Quaternion::Slerp(q1, q2, 0.5f);

    ASSERT_NEAR(result.w, 0.70710678f, 0.0001f); // cos(90°/2) = sqrt(2)/2
    ASSERT_NEAR(result.x, 0.70710678f, 0.0001f); // sin(90°/2) = sqrt(2)/2
    ASSERT_NEAR(result.y, 0.0f, 0.0001f);
    ASSERT_NEAR(result.z, 0.0f, 0.0001f);
}

TEST_CASE(Quaternion_ToMatrix_Rotation)
{
    Quaternion q = Quaternion::FromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 3.14159f); // 180° around Y-axis
    Matrix4 mat = q.ToMatrix();

    Vector3 original(1.0f, 0.0f, 0.0f);
    Vector3 rotated = mat * original; // Should rotate to (-1, 0, 0)

    ASSERT_NEAR(rotated.x, -1.0f, 0.0001f);
    ASSERT_NEAR(rotated.y, 0.0f, 0.0001f);
    ASSERT_NEAR(rotated.z, 0.0f, 0.0001f);
}

TEST_CASE(Quaternion_FromAxisAngle)
{
    Vector3 axis(0.0f, 1.0f, 0.0f); // Y-axis
    float angle = 3.14159f; // 180 degrees
    Quaternion q = Quaternion::FromAxisAngle(axis, angle);

    // Expected: w = cos(90°) = 0, xyz = (0, 1, 0) * sin(90°) = (0, 1, 0)
    ASSERT_NEAR(q.w, 0.0f, 0.0001f);
    ASSERT_NEAR(q.x, 0.0f, 0.0001f);
    ASSERT_NEAR(q.y, 1.0f, 0.0001f);
    ASSERT_NEAR(q.z, 0.0f, 0.0001f);
}
#pragma endregion

#endif