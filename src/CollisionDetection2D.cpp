#include "CollisionDetection2D.h"

using namespace MIRA;

AABB2D ComputeAABB2D(const Collider2D& collider)
{
	AABB2D aabb;
	if (collider.type == Collider2D::Type2D::CIRCLE)
	{
		aabb.max_x = collider.GetCircleRadius();
		aabb.max_y = aabb.max_x;
		aabb.min_x = -aabb.max_x;
		aabb.min_y = -aabb.max_x;
	}
	else // rect
	{
		aabb.max_x = collider.GetRectLength() / 2.0f;
		aabb.max_y = collider.GetRectHeight() / 2.0f;
		aabb.min_x = -aabb.max_x;
		aabb.min_y = -aabb.max_y;
	}
	return aabb;
}

bool CheckAABBOverlap2D(const AABB2D& a, const AABB2D& b)
{
	return (a.max_x > b.min_x && a.min_x < b.max_x) &&
		(a.max_y > b.min_y && a.min_y < b.max_y);
}

bool CheckCollision(const Collider2D& a, const Collider2D& b, CollisionInfo2D& info)
{
	if (a.type == Collider2D::Type2D::CIRCLE && b.type == Collider2D::Type2D::CIRCLE)
	{
		return CircleCircleCollision(a, b, info);
	}
	else if (a.type == Collider2D::Type2D::CIRCLE && b.type == Collider2D::Type2D::RECTANGLE)
	{
		return CircleRectCollision(a, b, info);
	}
	else if (a.type == Collider2D::Type2D::RECTANGLE && b.type == Collider2D::Type2D::RECTANGLE)
	{
		return RectRectCollision(a, b, info);
	}
	return false;
}

bool CircleCircleCollision(const Collider2D& a, const Collider2D& b, CollisionInfo2D& info)
{
	const Vector2 delta = b.body->position - a.body->position;
	const float distanceSq = delta.Dot(delta);
	const float combinedRad = a.GetCircleRadius() + b.GetCircleRadius();

	if (distanceSq > combinedRad * combinedRad) return false;

	if (distanceSq < 1e-6f) // Zero-distance case
	{
		info.normal = Vector2(1.0f, 0.0f); // Arbitrary
		info.depth = combinedRad;
		return true;
	}

	const float distance = std::sqrt(distanceSq);
	info.normal = delta / distance;
	info.depth = combinedRad - distance;

	return true;
}

bool CircleRectCollision(const Collider2D& circle, const Collider2D& rect, CollisionInfo2D& info)
{
	const Vector2 circlePos = circle.body->position;
	const float circleRadius = circle.GetCircleRadius();

	const Vector2 rectPos = rect.body->position;
	const float rectHalfLength = rect.GetRectLength() / 2.0f;
	const float rectHalfHeight = rect.GetRectHeight() / 2.0f;

	const Vector2 rectMin = rectPos - Vector2(rectHalfLength, rectHalfHeight);
	const Vector2 rectMax = rectPos + Vector2(rectHalfLength, rectHalfHeight);

	Vector2 closestPoint;
	closestPoint.x = std::max(rectMin.x, std::min(circlePos.x, rectMax.x));
	closestPoint.y = std::max(rectMin.y, std::min(circlePos.y, rectMax.y));

	const Vector2 delta = closestPoint - circlePos;
	const float distanceSq = delta.Dot(delta);

	if (distanceSq > circleRadius * circleRadius) return false;

	if (distanceSq < 1e-6f) // Circle center == closestPoint 
	{
		// Use direction from rect center to circle center
		Vector2 dir = circlePos - rectPos;
		if (dir.Dot(dir) < 1e-6f) // Centers coincide
			info.normal = Vector2(1.0f, 0.0f); // Default
		else
			info.normal = dir.Normalized();
		info.depth = circleRadius;
		return true;
	}

	const float distance = std::sqrt(distanceSq);
	info.normal = delta / distance;
	info.depth = circleRadius - distance;

	return true;
}

bool RectRectCollision(const Collider2D& a, const Collider2D& b, CollisionInfo2D& info)
{
	const Vector2 a_min = a.body->position - Vector2(a.GetRectLength() / 2.0f, a.GetRectHeight() / 2.0f);
	const Vector2 a_max = a.body->position + Vector2(a.GetRectLength() / 2.0f, a.GetRectHeight() / 2.0f);

	const Vector2 b_min = b.body->position - Vector2(b.GetRectLength() / 2.0f, b.GetRectHeight() / 2.0f);
	const Vector2 b_max = b.body->position + Vector2(b.GetRectLength() / 2.0f, b.GetRectHeight() / 2.0f);

	const bool overlapX = !(a_min.x >= b_max.x || a_max.x <= b_min.x);
	const bool overlapY = !(a_min.y >= b_max.y || a_max.y <= b_min.y);

	if (!overlapX || !overlapY) return false;

	const Vector2 overlap(
		std::min(a_max.x, b_max.x) - std::max(a_min.x, b_min.x),
		std::min(a_max.y, b_max.y) - std::max(a_min.y, b_min.y)
	);

	Vector2 dir = b.body->position - a.body->position;

	if (overlap.x < overlap.y)
	{
		info.depth = overlap.x;
		info.normal = (dir.x >= 0.0f) ? Vector2(1.0f, 0.0f) : Vector2(-1.0f, 0.0f);
	}
	else
	{
		info.depth = overlap.y;
		info.normal = (dir.y >= 0.0f) ? Vector2(0.0f, 1.0f) : Vector2(0.0f, -1.0f);
	}

	// Handle identical centers
	if (dir.Dot(dir) < 1e-6f)
	{
		info.normal = Vector2(1.0f, 0.0f); // Arbitrary default
	}

	return true;
}