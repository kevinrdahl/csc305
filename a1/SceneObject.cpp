#include "SceneObject.h"
#include "Ray.h"

using namespace RayTracer;
using namespace Eigen;

/*==============
 * SCENE OBJECT
 *=============*/
SceneObject::SceneObject(Vector3d* position, Vector3d* colour) {
	this->position = position;
	this->colour = colour;
}

//returns the intersection point and the normal, as a ray
//if it doesn't intersect, then NULL
Intersection* SceneObject::rayIntersect(Ray* ray) {
	return NULL;
}

void SceneObject::printName() {
	printf("BAD");
}

/*========
 * SPHERE
 *========*/
Sphere::Sphere(Vector3d* position, double radius, Vector3d* colour) : SceneObject(position, colour) {
	this->radius = radius;
}

//based on
//	http://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
//
Intersection* Sphere::rayIntersect(Ray* ray) {
	//the line from the ray's origin to the sphere's centre
	Vector3d originToCentre = *(this->position) - *(ray->origin);
	//printf("    originToCentre: [%f, %f, %f] (%f)\n", originToCentre(0), originToCentre(1), originToCentre(2), originToCentre.norm());

	//the point on the ray at which its distance to the sphere's centre is minimized
	//(this forms a right triangle with originToCentre)
	double rayLengthToMinimumDistance = originToCentre.dot(*(ray->direction));
	//printf("    rayLengthToMinimumDistance: %f\n", rayLengthToMinimumDistance);
	if (rayLengthToMinimumDistance < 0)
		return NULL;

	//the minimum distance between the ray and the sphere's centre
	//(b^2 = c^2 - a^2)
	double rayDistanceToCentre = sqrt(pow(originToCentre.norm(), 2) - pow(rayLengthToMinimumDistance, 2));
	//printf("    rayDistanceToCentre: %f\n", rayDistanceToCentre);
	if (rayDistanceToCentre > this->radius)
		return NULL;

	//the portion of the line segment which is inside the sphere
	double insideLength = sqrt(pow(this->radius, 2) - pow(rayDistanceToCentre, 2));
	//printf("    insideLength: %f\n", insideLength);

	//for this, only return the first point of intersection (don't render spheres which encompass the camera)
	if (insideLength > rayLengthToMinimumDistance)
		return NULL;

	double rayDistance = rayLengthToMinimumDistance - insideLength;

	Vector3d* intersect = new Vector3d(*(ray->origin) + *(ray->direction) * rayDistance);
	Vector3d* normal = new Vector3d((*intersect - *(this->position)).normalized());

	return new Intersection(this, intersect, normal);
}

void Sphere::printName() {
	printf("SPHERE");
}

/*========
* PLANE
*========*/
Plane::Plane(Vector3d* position, Vector3d* normal, Vector3d* colour) : SceneObject(position, colour) {
	this->normal = normal;
}

Intersection* Plane::rayIntersect(Ray* ray) {
	//dot product of the ray's direction and this plane's normal
	double rayPlaneDot = (*(ray->direction)).dot(*(this->normal));
	if (rayPlaneDot == 0)
		return NULL; //parallel, no intersection

	double rayDistance = (*(this->position) - *(ray->origin)).dot(*(this->normal)) / rayPlaneDot;
	if (rayDistance < 0)
		return NULL; //behind us, don't care
	//printf("Intersect with plane at %f\n", rayDistance);

	Vector3d* intersect = new Vector3d(*(ray->origin) + *(ray->direction) * rayDistance);
	Vector3d* normal = new Vector3d(*(this->normal));

	return new Intersection(this, intersect, normal);
}

void Plane::printName() {
	printf("PLANE");
}