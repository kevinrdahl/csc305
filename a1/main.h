#ifndef MAIN_H
#define MAIN_H

#include <Eigen\Dense>
#include <vector>
#include "Ray.h"
#include "SceneObject.h"

using namespace Eigen;

namespace RayTracer {
	std::vector<Intersection*>* getIntersections(std::vector<SceneObject*> objects, Ray* ray, SceneObject* ignore, bool any);
	Intersection* getClosestIntersection(Vector3d* point, std::vector<Intersection*>* intersections);
}

#endif