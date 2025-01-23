#include "MIRAMatrix.h"

using namespace MIRA;

Matrix4::Matrix4()
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            m[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

Matrix4 Matrix4::operator*(const Matrix4& other) const
{
    Matrix4 result;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            result.m[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                result.m[i][j] += m[i][k] * other.m[k][j];
            }
        }
    }
    return result;
}

Matrix4 Matrix4::Transpose() const
{
    Matrix4 result;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            result.m[i][j] = m[j][i];
        }
    }
    return result;
}

#pragma region statics
Matrix4 Matrix4::Translate(const Vector3& translation)
{
    Matrix4 result;
    result.m[0][3] = translation.x;
    result.m[1][3] = translation.y;
    result.m[2][3] = translation.z;
    return result;
}

Matrix4 Matrix4::RotateX(float angle)
{
    Matrix4 result;
    float cosA = std::cos(angle);
    float sinA = std::sin(angle);
    result.m[1][1] = cosA;
    result.m[1][2] = -sinA;
    result.m[2][1] = sinA;
    result.m[2][2] = cosA;
    return result;
}

Matrix4 Matrix4::RotateY(float angle)
{
    Matrix4 result;
    float cosA = std::cos(angle);
    float sinA = std::sin(angle);
    result.m[0][0] = cosA;
    result.m[0][2] = sinA;
    result.m[2][0] = -sinA;
    result.m[2][2] = cosA;
    return result;
}

Matrix4 Matrix4::RotateZ(float angle)
{
    Matrix4 result;
    float cosA = std::cos(angle);
    float sinA = std::sin(angle);
    result.m[0][0] = cosA;
    result.m[0][1] = -sinA;
    result.m[1][0] = sinA;
    result.m[1][1] = cosA;
    return result;
}

Matrix4 Matrix4::Scale(const Vector3& scale)
{
    Matrix4 result;
    result.m[0][0] = scale.x;
    result.m[1][1] = scale.y;
    result.m[2][2] = scale.z;
    return result;
}
#pragma endregion
