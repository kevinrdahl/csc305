#ifndef RAY_H
#define RAY_H

#include <Eigen\Dense>

using namespace Eigen;

namespace RayTracer {
	class SceneObject;

	class Ray {
	public:
		Ray(Vector3d* origin, Vector3d* direction);
		Vector3d* origin;
		Vector3d* direction;
	};

	class Intersection : public Ray {
	public:
		Intersection(SceneObject* object, Vector3d* origin, Vector3d* direction);
		SceneObject* object;
	};
}

#endif