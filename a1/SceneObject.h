#ifndef SCENEOBJECT_H
#define SCENEOBJECT_H

#include <Eigen\Dense>
#include "Ray.h"

using namespace Eigen;

namespace RayTracer {
	class SceneObject {
	public:
		SceneObject(Vector3d* position, Vector3d* colour);
		Vector3d* position;
		Vector3d* colour;
		double reflectivity = 0;
		virtual Intersection* rayIntersect(Ray* ray);
		virtual void printName();
	};

	class Sphere : public SceneObject {
	public:
		double radius;

		Sphere(Vector3d* position, double radius, Vector3d* colour);
		Intersection* rayIntersect(Ray* ray);
		void printName();
	};

	class Plane : public SceneObject {
	public:
		Vector3d* normal;

		Plane(Vector3d* position, Vector3d* normal, Vector3d* colour);
		Intersection* rayIntersect(Ray* ray);
		void printName();
	};
}

#endif