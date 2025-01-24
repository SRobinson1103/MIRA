﻿#include "MIRAVector.h"

#include <stdexcept>


using namespace MIRA;

MIRA::Vector3::Vector3() : x(0.0f), y(0.0f), z(0.0f) {}

MIRA::Vector3::Vector3(float scalar) : x(scalar), y(scalar), z(scalar) {}

Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

Vector3 Vector3::operator+(const Vector3& other) const { return Vector3(x + other.x, y + other.y, z + other.z); }

Vector3 Vector3::operator-(const Vector3& other) const { return Vector3(x - other.x, y - other.y, z - other.z); }

Vector3 Vector3::operator*(float scalar) const { return Vector3(x * scalar, y * scalar, z * scalar); }

Vector3 Vector3::operator/(float scalar) const { return Vector3(x / scalar, y / scalar, z / scalar); }

float& Vector3::operator[](int index)
{
    switch (index)
    {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    default: throw std::out_of_range("Vector3 index out of range!");
    }
}

const float& Vector3::operator[](int index) const
{
    switch (index)
    {
    case 0: return x;
    case 1: return y;
    case 2: return z;
    default: throw std::out_of_range("Vector3 index out of range!");
    }
}

Vector3 Vector3::Abs() const { return Vector3(std::fabs(x), std::fabs(y), std::fabs(z)); }

float Vector3::Dot(const Vector3& other) const { return x * other.x + y * other.y + z * other.z; }

Vector3 Vector3::Cross(const Vector3& other) const
{
    return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

float Vector3::Magnitude() const { return std::sqrt(x * x + y * y + z * z); }

Vector3 Vector3::Normalized() const
{
    float mag = Magnitude();
    if (mag > 0.0f)
    {
        return *this / mag;
    }
    return Vector3(); // Return zero vector if magnitude is zero
}

#pragma region statics
float Vector3::Distance(const Vector3& a, const Vector3& b) { return (a - b).Magnitude(); }

float Vector3::Angle(const Vector3& a, const Vector3& b)
{
    float dot = a.Dot(b);
    float magA = a.Magnitude();
    float magB = b.Magnitude();
    return std::acos(dot / (magA * magB));
}
#pragma endregion

Vector3 MIRA::operator*(float scalar, const Vector3& vec) { return vec * scalar; }
