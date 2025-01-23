#ifndef MIRA_MATRIX_H
#define MIRA_MATRIX_H

#include "MIRAVector.h"

namespace MIRA
{
class Matrix4
{
public:
    float m[4][4]; // 4x4 matrix

    // Constructor (initializes to identity matrix)
    Matrix4();

    // Matrix multiplication
    Matrix4 operator*(const Matrix4& other) const;

    // Transpose the matrix
    Matrix4 Transpose() const;

    // Create a translation matrix
    static Matrix4 Translate(const Vector3& translation);

    // Create a rotation matrix (around X-axis)
    static Matrix4 RotateX(float angle);

    // Create a rotation matrix (around Y-axis)
    static Matrix4 RotateY(float angle);

    // Create a rotation matrix (around Z-axis)
    static Matrix4 RotateZ(float angle);

    // Create a scaling matrix
    static Matrix4 Scale(const Vector3& scale);
};
}//End namespace

#endif