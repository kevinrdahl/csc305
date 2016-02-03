#include "Image.h"
#include <Eigen\Dense>
#include <vector>

#include "main.h"
#include "Ray.h"
#include "SceneObject.h"

using namespace Eigen;
using namespace std;
using namespace RayTracer;

namespace RayTracer {
	vector<Intersection*>* getIntersections(vector<SceneObject*>* objects, Ray* ray, SceneObject* ignore, bool any) {
		unsigned int i;
		SceneObject* obj;

		vector<Intersection*>* v = new vector<Intersection*>();

		for (i = 0; i < objects->size(); i++) {
			obj = (*objects)[i];
			if (obj == ignore)
				continue;
			Intersection* intersect = obj->rayIntersect(ray);
			if (intersect != NULL) {
				v->push_back(intersect);
				if (any)
					break;
			}
		}

		return v;
	}

	Intersection* getClosestIntersection(Vector3d* point, vector<Intersection*>* intersections) {
		if (intersections->size() == 0)
			return NULL;

		double smallestDistance = DBL_MAX;
		Intersection* closest = NULL;
		Intersection* intersection = NULL;
		unsigned int i = 0;

		for (i = 0; i < intersections->size(); i++) {
			intersection = (*intersections)[i];
			double distance = (*(intersection->origin) - *point).norm();
			if (distance < smallestDistance) {
				smallestDistance = distance;
				closest = intersection;
			}
		}

		return closest;
	}

	Vector3d traceRay(Ray* ray, vector<SceneObject*>* objects, vector<SceneObject*>* lights, int remainingDepth) {
		Vector3d backgroundColour(0, 0, 0);
		Vector3d ambientLight(25, 25, 25);

		/*Vector3d RED(255, 0, 0);
		Vector3d GREEN(0, 255, 0);
		Vector3d BLUE(255, 0, 255);
		Vector3d YELLOW(255, 255, 0);
		Vector3d CYAN(0, 255, 255);
		Vector3d MAJENTA(255, 0, 255);*/

		if (remainingDepth <= 0)
			return backgroundColour;

		vector<Intersection*>* intersections = getIntersections(objects, ray, NULL, false);
		Intersection* closestIntersection = getClosestIntersection(ray->origin, intersections);
		intersections->clear();
		delete intersections;

		if (closestIntersection != NULL) {
			Vector3d fullLightColour = ambientLight;
			Vector3d surfaceColour = *(closestIntersection->object->colour);

			//for each light, add it to the full light on this point (if not blocked)
			for (unsigned int lightNum = 0; lightNum < lights->size(); lightNum++) {
				SceneObject* light = (*lights)[lightNum];
				Vector3d toLight = *(light->position) - *(closestIntersection->origin);
				Vector3d toLightNormalized = toLight.normalized();
				double dot = toLightNormalized.dot(*(closestIntersection->direction));

				if (dot > 0) {
					intersections = getIntersections(objects, new Ray(closestIntersection->origin, &toLightNormalized), closestIntersection->object, true);
					bool inLight = false;
					if (intersections->size() == 0) {
						inLight = true;
					}
					else {
						Intersection* intersection = (*intersections)[0];
						SceneObject* obj = intersection->object;
						intersections->clear();

						if ((*(intersection->origin) - *(closestIntersection->origin)).norm() > toLight.norm()) {
							inLight = true;
						}
					}

					if (inLight) {
						fullLightColour += *(light->colour) * dot;
					}

					delete intersections;
				}
			}

			double reflectivity = closestIntersection->object->reflectivity;
			if (reflectivity > 0) {
				Vector3d rayDirection = *(ray->direction);
				Vector3d normal = *(closestIntersection->direction);
				Vector3d reflectedDirection = rayDirection - ((2 * (normal.dot(rayDirection))) * normal);
				Ray* reflectedRay = new Ray(closestIntersection->origin, &reflectedDirection);

				Vector3d reflectionColour = traceRay(reflectedRay, objects, lights, remainingDepth - 1);
				delete reflectedRay;

				surfaceColour *= 1 - reflectivity;
				surfaceColour += reflectionColour * reflectivity;
			}

			Vector3d endColour = surfaceColour;
			endColour[0] *= fullLightColour[0] / 255;
			endColour[1] *= fullLightColour[1] / 255;
			endColour[2] *= fullLightColour[2] / 255;

			return endColour;
		}
		else {
			return backgroundColour;
		}
	}
}

void printVector(Vector3d* v, int numSpaces, bool newLine) {
	for (int i = 0; i < numSpaces; i++)
		printf(" ");

	printf("[%f, %f, %f]", 
		(*v)[0],
		(*v)[1],
		(*v)[2]
	);

	if (newLine)
		printf("\n");
}

Pixel getPixel(Vector3d Colour)
{
	Pixel px(0,0,0);

	if (Colour(0) < 0) px.R = 0;
	else if (Colour(0) > 255) px.R = 255;
	else px.R = (unsigned char)Colour(0);

	if (Colour(1) < 0) px.G = 0;
	else if (Colour(1) > 255) px.G = 255;
	else px.G = (unsigned char)Colour(1);

	if (Colour(2) < 0) px.B = 0;
	else if (Colour(2) > 255) px.B = 255;
	else px.B = (unsigned char)Colour(2);

	px.A = 255;

	return px;
}

Vector3d* vectorFromPixel(Pixel* px) {
	return new Vector3d(px->R, px->G, px->B);
}

int main(int, char**) {
	unsigned int imageWidth = 600;
	unsigned int imageHeight = 600;

	unsigned int width = 0;
	unsigned int height = 0;
	unsigned int x, y;

	unsigned int supersampling = 2;
	if (supersampling % 2 == 1)
		supersampling = 1;

	width = imageWidth * supersampling;
	height = imageHeight * supersampling;

	Vector3d** pixelColours = new Vector3d*[width];
	for (unsigned int i = 0; i < width; i++) {
		pixelColours[i] = new Vector3d[height];
	}

	Image image(imageWidth, imageHeight);

	vector<SceneObject*>* lights = new vector<SceneObject*>();
	vector<SceneObject*>* objects = new vector<SceneObject*>();
	Vector3d cameraPosition(256, 256, -1000);

	Sphere* sphere1 = new Sphere(new Vector3d(200, 300, 550), 300, new Vector3d(255, 50, 50));
	Sphere* sphere2 = new Sphere(new Vector3d(600, 100, 500), 100, new Vector3d(100, 255, 100));
	sphere1->reflectivity = 0.90;

	Vector3d planeNormal(0, 1, 0);
	planeNormal = planeNormal.normalized();
	Plane* plane1 = new Plane(
		new Vector3d(0, 0, 0),
		new Vector3d(planeNormal),
		new Vector3d(100, 100, 100)
	);
	//plane1->reflectivity = 0.9;

	planeNormal = Vector3d(-1.5, 0.5, -1);
	planeNormal = planeNormal.normalized();
	Plane* plane2 = new Plane(
		new Vector3d(1000, 0, 5000),
		new Vector3d(planeNormal),
		new Vector3d(100, 100, 100)
	);

	objects->push_back(sphere1);
	objects->push_back(sphere2);
	objects->push_back(plane1);
	objects->push_back(plane2);

	SceneObject* light1 = new SceneObject(new Vector3d(-300, 300, 0), new Vector3d(250, 100, 100));
	SceneObject* light2 = new SceneObject(new Vector3d(800, 500, -1000), new Vector3d(100, 100, 250));
	lights->push_back(light1);
	lights->push_back(light2);

	Vector3d cameraTopLeft = cameraPosition;
	cameraTopLeft(0) = 0;//-= width / 2;
	cameraTopLeft(1) = 0;//-= height / 2;

	bool abortLoop = false;

	for (x = 0; x < width && !abortLoop; x++) {
		printf("line %d\n", x);
		for (y = 0; y < height && !abortLoop; y++) {
			bool testOverride = false;

			//cast a ray!
			Vector3d rayOrigin;
			rayOrigin = Vector3d(x / (double)supersampling, y / (double)supersampling, 0);

			Vector3d rayDirection = (rayOrigin - cameraPosition).normalized();
			Ray ray(&rayOrigin, &rayDirection);

			pixelColours[x][y] = traceRay(&ray, objects, lights, 2);
		}
	}

	for (x = 0; x < imageWidth; x++) {
		for (y = 0; y < imageHeight; y++) {
			if (supersampling > 1) {
				Vector3d colour(0, 0, 0);
				unsigned int x2, y2;
				for (x2 = 0; x2 < supersampling; x2++) {
					for (y2 = 0; y2 < supersampling; y2++) {
						colour += pixelColours[x*2 + x2][y*2 + y2];
					}
				}
				colour /= (supersampling * supersampling);

				image(x, imageHeight - y - 1) = getPixel(colour);
			}
			else {
				image(x, imageHeight - y - 1) = getPixel(pixelColours[x][y]);
			}
		}
	}

	image.save("C:\\Users\\Kevin\\Desktop\\raytracer.png");
	image.show("Ray Tracer");
}