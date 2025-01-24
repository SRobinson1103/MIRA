#include "MIRAMatrix.h"

using namespace MIRA;

Matrix4::Matrix4()
{
    memset(m, 0, 16 * sizeof(float));
    m[0][0] = 1.0f;
    m[1][1] = 1.0f;
    m[2][2] = 1.0f;
    m[3][3] = 1.0f;
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

Matrix4 Matrix4::Inverse() const
{
    Matrix4 result;
    float temp[4][8];

    // Initialize augmented matrix
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            temp[i][j] = m[i][j];
            temp[i][j + 4] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Perform Gauss-Jordan elimination
    for (int i = 0; i < 4; ++i)
    {
        // Find the pivot
        float pivot = temp[i][i];
        if (std::fabs(pivot) < 1e-6f)
        {
            // Matrix is not invertible
            return Matrix4();
        }

        // Normalize the pivot row
        for (int j = 0; j < 8; ++j)
        {
            temp[i][j] /= pivot;
        }

        // Eliminate other rows
        for (int k = 0; k < 4; ++k)
        {
            if (k != i)
            {
                float factor = temp[k][i];
                for (int j = 0; j < 8; ++j)
                {
                    temp[k][j] -= factor * temp[i][j];
                }
            }
        }
    }

    // Extract the inverse matrix
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            result.m[i][j] = temp[i][j + 4];
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

Vector3 MIRA::operator*(const Matrix4& mat, const Vector3& vec)
{
    Vector3 result;
    result.x = mat.m[0][0] * vec.x + mat.m[0][1] * vec.y + mat.m[0][2] * vec.z + mat.m[0][3] * 1.0f;
    result.y = mat.m[1][0] * vec.x + mat.m[1][1] * vec.y + mat.m[1][2] * vec.z + mat.m[1][3] * 1.0f;
    result.z = mat.m[2][0] * vec.x + mat.m[2][1] * vec.y + mat.m[2][2] * vec.z + mat.m[2][3] * 1.0f;
    return result;
}
