#ifndef MIRA_VECTOR_H
#define MIRA_VECTOR_H

namespace MIRA
{
    struct Vector3
    {
        float x, y, z;
        Vector3 operator+(const Vector3& other) const
        {
            return { x + other.x, y + other.y, z + other.z };
        }

        float Dot(const Vector3& other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }
    };
}

#endif