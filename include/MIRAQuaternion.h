#ifndef MIRA_QUATERNION_H
#define MIRA_QUATERNION_H

#include "MIRAMatrix.h"

namespace MIRA
{
class Quaternion
{
public:
    float w, x, y, z;

    Quaternion(float w = 1.0f, float x = 0.0f, float y = 0.0f, float z = 0.0f);

    // Addition
    Quaternion operator+(const Quaternion& other) const;

    // Subtraction
    Quaternion operator-(const Quaternion& other) const;

    // Multiplication
    Quaternion operator*(const Quaternion& other) const;

    // Conjugate
    Quaternion Conjugate() const;

    // Inverse
    Quaternion Inverse() const;

    // Normalization
    Quaternion Normalized() const;

    // Dot product
    float Dot(const Quaternion& other) const;

    // Convert to rotation matrix
    Matrix4 ToMatrix() const;

    // Spherical interpolation
    static Quaternion Slerp(const Quaternion& a, const Quaternion& b, float t);

    static Quaternion FromAxisAngle(const Vector3& axis, float angle);
};
} //End namespace

#endif