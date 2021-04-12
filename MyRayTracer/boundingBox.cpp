#ifndef AABB_H
#define AABB_H

#include "vector.h"
#include "boundingBox.h"

//-------------------------------------------------------------------- - default constructor
AABB::AABB(void) 
{
	min = Vector(-1.0f, -1.0f, -1.0f);
	max = Vector(1.0f, 1.0f, 1.0f);
}

// --------------------------------------------------------------------- constructor
AABB::AABB(const Vector& v0, const Vector& v1)
{
	min = v0; max = v1;
}

// --------------------------------------------------------------------- copy constructor
AABB::AABB(const AABB& bbox) 
{
	min = bbox.min; max = bbox.max;
}

// --------------------------------------------------------------------- assignment operator
AABB AABB::operator= (const AABB& rhs) {
	if (this == &rhs)
		return (*this);
	min = rhs.min;
	max = rhs.max;
	return (*this);
}

// --------------------------------------------------------------------- destructor
AABB::~AABB() {}

// -------------------------------------------------------------------- - inside
// used to test if a ray starts inside a bbox

bool AABB::isInside(const Vector & p)
{
	return ((p.x > min.x && p.x < max.x) && (p.y > min.y && p.y < max.y) && (p.z > min.z && p.z < max.z));
}

bool AABB::includes(const AABB& aabb)
{
	return (
		aabb.max.x <= max.x &&
		aabb.max.y <= max.y &&
		aabb.max.z <= max.z &&
		aabb.min.x >= min.x &&
		aabb.min.y >= min.y &&
		aabb.min.z >= min.z
	);
}

// --------------------------------------------------------------------- compute centroid
Vector AABB::centroid(void) {
	return (min + max) / 2;
}

// --------------------------------------------------------------------- extend AABB
void AABB::extend(AABB box) {
	if (min.x > box.min.x) min.x = box.min.x;
	if (min.y > box.min.y) min.y = box.min.y;
	if (min.z > box.min.z) min.z = box.min.z;

	if (max.x < box.max.x) max.x = box.max.x;
	if (max.y < box.max.y) max.y = box.max.y;
	if (max.z < box.max.z) max.z = box.max.z;
}

// --------------------------------------------------------------------- AABB intersection


bool AABB::intercepts(Ray& r, float& t)
{
	float ox = r.origin.x, oy = r.origin.y, oz = r.origin.z;
	float dx = r.direction.x, dy = r.direction.y, dz = r.direction.z;

	float tx_min, ty_min, tz_min;
	float tx_max, ty_max, tz_max;

	float a = 1.0 / dx;
	if (a >= 0) {
		tx_min = (min.x - ox) * a;
		tx_max = (max.x - ox) * a;
	}
	else {
		tx_min = (max.x - ox) * a;
		tx_max = (min.x - ox) * a;
	}

	float b = 1.0 / dy;
	if (b >= 0) {
		ty_min = (min.y - oy) * b;
		ty_max = (max.y - oy) * b;
	}
	else {
		ty_min = (max.y - oy) * b;
		ty_max = (min.y - oy) * b;
	}

	float c = 1.0 / dz;
	if (c >= 0) {
		tz_min = (min.z - oz) * c;
		tz_max = (max.z - oz) * c;
	}
	else {
		tz_min = (max.z - oz) * c;
		tz_max = (min.z - oz) * c;
	}

	float t0, t1;

	//largest entering t value
	t0 = MAX3(tx_min, ty_min, tz_min);

	//smallest exiting t value
	t1 = MIN3(tx_max, ty_max, tz_max);

	t = (t0 < 0) ? t1 : t0;

	return (t0 < t1&& t1 > 0);
}

#endif