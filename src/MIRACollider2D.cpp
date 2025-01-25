#include "MIRACollider2D.h"

using namespace MIRA;

Collider2D::Collider2D(RigidBody2D* body, Type2D type) : type(type), body(body)
{
	if (type == CIRCLE)
	{
		circle.radius = 0.0f;
	}
	else
	{
		rect.length = 0.0f;
		rect.height = 0.0f;
	}
}