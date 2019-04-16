#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  Vector3D centreToPoint = pm.position - origin;
  if (centreToPoint.norm() <= radius){
    centreToPoint.normalize();
    Vector3D tangentPoint = origin + centreToPoint*radius;
    Vector3D correctionVector = tangentPoint - pm.last_position;
    pm.position = pm.last_position + correctionVector*(1-friction);
  } else return;
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}