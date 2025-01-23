#include "MIRAVector.h"

using namespace MIRA;

Vector3::Vector3(float x = 0.0f, float y = 0.0f, float z = 0.0f) : x(x), y(y), z(z) {}

Vector3 Vector3::operator+(const Vector3& other) const { return Vector3(x + other.x, y + other.y, z + other.z); }

Vector3 Vector3::operator-(const Vector3& other) const { return Vector3(x - other.x, y - other.y, z - other.z); }

Vector3 Vector3::operator*(float scalar) const { return Vector3(x * scalar, y * scalar, z * scalar); }

Vector3 Vector3::operator/(float scalar) const { return Vector3(x / scalar, y / scalar, z / scalar); }

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
