#include "sampler.h"

// Sampling with rejection method
Vector sample_unit_disk(void) {
	Vector p;
	do {
		p = Vector(rand_float(), rand_float(), 0.0) * 2 - Vector(1.0, 1.0, 0.0);
	} while (p*p >= 1.0);
	return p;
}

// Random vector inside a sphere
Vector rand_in_unit_sphere(void) {
	Vector point = Vector(rand_float(), rand_float(), rand_float());
	point.normalize();
	float d = rand_float();
	point *= d;
	return point;
}