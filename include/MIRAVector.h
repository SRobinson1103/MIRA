#ifndef MIRA_VECTOR_H
#define MIRA_VECTOR_H

#include <cmath>

namespace MIRA
{
class Vector3
{
public:
    float x, y, z;

    Vector3(float x, float y, float z);

    // Addition
    Vector3 operator+(const Vector3& other) const;

    // Subtraction
    Vector3 operator-(const Vector3& other) const;

    // Scalar multiplication
    Vector3 operator*(float scalar) const;

    // Scalar division
    Vector3 operator/(float scalar) const;

    // Dot product
    float Dot(const Vector3& other) const;

    // Cross product
    Vector3 Cross(const Vector3& other) const;

    // Magnitude of the vector
    float Magnitude() const;

    // Normalize the vector
    Vector3 Normalized() const;

    // Distance between two vectors
    static float Distance(const Vector3& a, const Vector3& b);

    // Angle between two vectors (in radians)
    static float Angle(const Vector3& a, const Vector3& b);
};
} //End Namespace

#endif