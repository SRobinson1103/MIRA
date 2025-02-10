#ifndef MIRA_COLLIDER2D_H
#define MIRA_COLLIDER2D_H

#include "MIRARigidBody2D.h"

#include <cassert>

namespace MIRA
{
struct AABB2D
{
    float min_x, max_x, min_y, max_y;
    AABB2D() : min_x(0.0f), min_y(0.0f), max_x(0.0f), max_y(0.0f) {}
    AABB2D(float minX, float minY, float maxX, float maxY) : min_x(minX), min_y(maxX), max_x(minY), max_y(maxY) {}
};

// Avoids polymorphism for slight performance gains
class Collider2D
{
public:
    int id;
    enum Type2D { CIRCLE, RECTANGLE};
    Type2D type;
    RigidBody2D* body;
    AABB2D bounds;

    Collider2D(int id, RigidBody2D* body, Type2D type);
    Collider2D(int id, RigidBody2D* body, Type2D type, AABB2D& aabb);

private:
    union
    {
        struct { float radius; } circle;
        struct { float length;
                    float height; } rect;
    };

public:
    // Debug only accessors and setters with validation
#if !defined(NDEBUG)
    float GetCircleRadius() const
    {
        assert(type == CIRCLE && "Collider is not a circle!");
        return circle.radius;
    }

    float GetRectLength() const
    {
        assert(type == RECTANGLE && "Collider is not a rectangle!");
        return rect.length;
    }
    float GetRectHeight() const
    {
        assert(type == RECTANGLE && "Collider is not a rectangle!");
        return rect.height;
    }

    void SetCircleRadius(float radius)
    {
        assert(type == CIRCLE && "Collider is not a circle!");
        circle.radius = radius;
    }

    void SetRectLength(float length)
    {
        assert(type == RECTANGLE && "Collider is not a rectangle!");
        rect.length = length;
    }

    void SetRectHeight(float height)
    {
        assert(type == RECTANGLE && "Collider is not a rectangle!");
        rect.height = height;
    }
#else // Release
        float GetCircleRadius() const { return circle.radius; }
        float GetRectLength() const { return rect.length; }
        float GetRectHeight() const { return rect.height; }
        void SetCircleRadius(float radius) { circle.radius = radius; }
        void SetRectLength(float length) { rect.length = length; }
        void SetRectHeight(float height) { rect.height = height; }
#endif
    };
}

#endif
