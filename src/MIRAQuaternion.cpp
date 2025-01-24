#include "MIRAQuaternion.h"

using namespace MIRA;

Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

Quaternion Quaternion::operator+(const Quaternion& other) const { return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z); }

Quaternion Quaternion::operator-(const Quaternion& other) const { return Quaternion(w - other.w, x - other.x, y - other.y, z - other.z); }

Quaternion Quaternion::operator*(const Quaternion& other) const
{
    return Quaternion(
        w * other.w - x * other.x - y * other.y - z * other.z,
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w
    );
}

Quaternion Quaternion::Conjugate() const { return Quaternion(w, -x, -y, -z); }

Quaternion Quaternion::Inverse() const
{
    float normSquared = w * w + x * x + y * y + z * z;
    if (normSquared > 0.0f)
    {
        float invNormSquared = 1.0f / normSquared;
        return Conjugate() * invNormSquared;
    }
    return Quaternion(); // Return identity if norm is zero
}

Quaternion Quaternion::Normalized() const
{
    float norm = std::sqrt(w * w + x * x + y * y + z * z);
    if (norm > 0.0f)
    {
        return Quaternion(w / norm, x / norm, y / norm, z / norm);
    }
    return Quaternion(); // Return identity if norm is zero
}

float Quaternion::Dot(const Quaternion& other) const { return w * other.w + x * other.x + y * other.y + z * other.z; }

Matrix4 Quaternion::ToMatrix() const
{
    Matrix4 mat;
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    mat.m[0][0] = 1.0f - 2.0f * (yy + zz);
    mat.m[0][1] = 2.0f * (xy - wz);
    mat.m[0][2] = 2.0f * (xz + wy);
    mat.m[0][3] = 0.0f;

    mat.m[1][0] = 2.0f * (xy + wz);
    mat.m[1][1] = 1.0f - 2.0f * (xx + zz);
    mat.m[1][2] = 2.0f * (yz - wx);
    mat.m[1][3] = 0.0f;

    mat.m[2][0] = 2.0f * (xz - wy);
    mat.m[2][1] = 2.0f * (yz + wx);
    mat.m[2][2] = 1.0f - 2.0f * (xx + yy);
    mat.m[2][3] = 0.0f;

    mat.m[3][0] = 0.0f;
    mat.m[3][1] = 0.0f;
    mat.m[3][2] = 0.0f;
    mat.m[3][3] = 1.0f;

    return mat;
}

#pragma region statics
Quaternion Quaternion::Slerp(const Quaternion& a, const Quaternion& b, float t)
{
    float dot = a.Dot(b);
    float theta = std::acos(dot);
    float sinTheta = std::sin(theta);

    if (sinTheta > 0.001f)
    {
        float w1 = std::sin((1.0f - t) * theta) / sinTheta;
        float w2 = std::sin(t * theta) / sinTheta;
        return a * w1 + b * w2;
    }
    else
    {
        // Fallback to linear interpolation if theta is small
        return a * (1.0f - t) + b * t;
    }
}

Quaternion Quaternion::FromAxisAngle(const Vector3& axis, float angle)
{
    float halfAngle = angle * 0.5f;
    float sinHalf = std::sin(halfAngle);
    float cosHalf = std::cos(halfAngle);

    Vector3 normalizedAxis = axis.Normalized();
    return Quaternion(
        cosHalf,
        normalizedAxis.x * sinHalf,
        normalizedAxis.y * sinHalf,
        normalizedAxis.z * sinHalf
    );
}
#pragma endregion