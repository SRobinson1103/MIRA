#ifndef MIRA_VECTOR_H
#define MIRA_VECTOR_H

#include <cmath>

namespace MIRA
{
class Vector3
{
public:
    float x, y, z;

    Vector3();

    Vector3(float scalar);

    Vector3(float x, float y, float z);

    // Addition
    Vector3 operator+(const Vector3& other) const;

    // Subtraction
    Vector3 operator-(const Vector3& other) const;

    // Scalar multiplication
    Vector3 operator*(float scalar) const;

    // Scalar division
    Vector3 operator/(float scalar) const;

    // Subscript operator for read/write access
    float& operator[](int index);

    // Const version for read-only access
    const float& operator[](int index) const;

    // Overloading negation operator
    Vector3 operator-() const;

    // absolute value of each component
    Vector3 Abs() const;

    // Dot product
    float Dot(const Vector3& other) const;

    // Cross product
    Vector3 Cross(const Vector3& other) const;

    // Magnitude of the vector
    float Magnitude() const;

    // Normalize the vector
    Vector3 Normalized() const;

    // Unit vector
    Vector3 Unit() const;

    // Axis with the smallest value
    int MinAxis() const;

    // Distance between two vectors
    static float Distance(const Vector3& a, const Vector3& b);

    // Angle between two vectors (in radians)
    static float Angle(const Vector3& a, const Vector3& b);
};

// Non-member function for float * Vector3
Vector3 operator*(float scalar, const Vector3& vec);
} //End Namespace

#endif