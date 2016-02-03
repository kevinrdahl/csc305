#include "Ray.h"

using namespace RayTracer;
using namespace Eigen;

Ray::Ray(Vector3d* origin, Vector3d* direction) {
	this->origin = origin;
	this->direction = direction;
}

Intersection::Intersection(SceneObject* object, Vector3d* origin, Vector3d* direction) : Ray(origin, direction) {
	this->object = object;
}