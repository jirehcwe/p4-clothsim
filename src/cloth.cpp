#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.

  double heightSpacing = height/(double)(num_height_points-1);
  double widthSpacing = width/(double)(num_width_points-1);


  //Creating point masses
  for (int y = 0; y < num_height_points; y++){
    for (int x = 0; x < num_width_points; x++){
      Vector3D pos;
      if (orientation == VERTICAL){
        double offset = ((double)(rand() % 2) - 1.0)/1000.0;
        pos = Vector3D(x*widthSpacing, y*heightSpacing, offset);
      } else {
        pos = Vector3D(x*widthSpacing, 1, y*heightSpacing);
      }
      PointMass mass = PointMass(pos,false);
      point_masses.emplace_back(mass);
    }
  }

  //Allocating pinned points
  for (int i = 0; i < pinned.size(); i++){
    point_masses[pinned[i][1] * num_width_points + pinned[i][0]].pinned = true;
  }

  // Creating springs
  for (int y = 0; y < num_height_points; y++){
    for (int x = 0; x < num_width_points; x++){
      //Structural: left, and above
      if (x-1 >= 0){
        Spring structuralSpringLeft = Spring(&point_masses[y*num_width_points + x], &point_masses[ y * num_width_points + (x-1)], STRUCTURAL);
        springs.emplace_back(structuralSpringLeft);
      }
      if (y+1 < num_height_points){
        Spring structuralSpringAbove = Spring(&point_masses[y*num_width_points + x], &point_masses[ (y+1) * num_width_points + x ], STRUCTURAL);
        springs.emplace_back(structuralSpringAbove);
      }

      //Shearing: diagonal upper left and upper right
      if (x-1 >= 0 && y-1 >= 0){
        Spring shearSpringDiagLeft = Spring(&point_masses[y*num_width_points + x], &point_masses[ (y-1) * num_width_points + (x-1)], SHEARING);
        springs.emplace_back(shearSpringDiagLeft);
      }
      if (x+1 < num_width_points && y-1 >= 0){
        Spring shearSpringDiagRight = Spring(&point_masses[y*num_width_points + x], &point_masses[ (y-1) * num_width_points + (x+1) ], SHEARING);
        springs.emplace_back(shearSpringDiagRight);
      }

      //Bending: two left, two above
      if (x-2 >= 0){
        Spring bendingSpringLeft = Spring(&point_masses[y*num_width_points + x], &point_masses[ y * num_width_points + (x-2)], BENDING);
        springs.emplace_back(bendingSpringLeft);
      }
      if (y-2 >= 0){
        Spring bendingSpringAbove = Spring(&point_masses[y*num_width_points + x], &point_masses[ (y-2) * num_width_points + x ], BENDING);
        springs.emplace_back(bendingSpringAbove);
      }      
    }
  }
  
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.

  //Reset forces
  for (int y = 0; y < num_height_points; y++){
    for (int x = 0; x < num_width_points; x++){
      int currentMass = y * num_width_points + x;
      point_masses[currentMass].forces = 0;
    }
  }

  // External forces
  for (int y = 0; y < num_height_points; y++){
    for (int x = 0; x < num_width_points; x++){
      int currentMass = y * num_width_points + x;

      Vector3D externalForce = external_accelerations[0] * mass;
      point_masses[currentMass].forces += externalForce;
    }
  }

  // Calculate forces on each point by springs
  for (int s = 0; s < springs.size(); s++){
    Spring *currentSpring = &springs[s];

    switch(currentSpring->spring_type){
      case STRUCTURAL:
        if (cp->enable_structural_constraints){

          Vector3D forceVector = currentSpring->pm_b->position-currentSpring->pm_a->position;
          double distance = forceVector.norm() - currentSpring->rest_length;
          forceVector.normalize();

          currentSpring->pm_a->forces +=  cp->ks*distance*forceVector;
          currentSpring->pm_b->forces += -cp->ks*distance*forceVector;
        }
        break;
      case SHEARING:
        if (cp->enable_shearing_constraints){
          Vector3D forceVector = currentSpring->pm_b->position-currentSpring->pm_a->position;
          double distance = forceVector.norm() - currentSpring->rest_length;
          forceVector.normalize();

          currentSpring->pm_a->forces +=  cp->ks*distance*forceVector;
          currentSpring->pm_b->forces += -cp->ks*distance*forceVector;
        } 
        break;
      case BENDING:
        if (cp->enable_bending_constraints){
          Vector3D forceVector = currentSpring->pm_b->position-currentSpring->pm_a->position;
          double distance = forceVector.norm() - currentSpring->rest_length;
          forceVector.normalize();

          currentSpring->pm_a->forces +=  0.2f*cp->ks*distance*forceVector;
          currentSpring->pm_b->forces += -0.2f*cp->ks*distance*forceVector;
        }
        break;
      

    }
      

      

      
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (int y = 0; y < num_height_points; y++){
    for (int x = 0; x < num_width_points; x++){
      
      int currentMass = y * num_width_points + x;

      if(!point_masses[currentMass].pinned){

      double dampingConstant = 1 - cp->damping/100.0;
      Vector3D positionDifference = point_masses[currentMass].position - point_masses[currentMass].last_position;
      Vector3D displacementFromForces = point_masses[currentMass].forces * delta_t * delta_t/mass;

      Vector3D newPos = point_masses[currentMass].position + dampingConstant*positionDifference + displacementFromForces;

      point_masses[currentMass].last_position = point_masses[currentMass].position;
      point_masses[currentMass].position = newPos;
      }
    }
  }


  // TODO (Part 4): Handle self-collisions.

  build_spatial_map();

  for (int p = 0; p < point_masses.size(); p++){
    self_collide(point_masses[p], simulation_steps);
  }

  

  // TODO (Part 3): Handle collisions with other primitives.

  for (int y = 0; y < num_height_points; y++){
    for (int x = 0; x < num_width_points; x++){
      int currentMass = y * num_width_points + x;
        for (int i = 0; i < collision_objects->size(); i++){
        collision_objects->at(i)->collide(point_masses[currentMass]);
      }
    }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].

  for (int s = 0; s < springs.size(); s++){
    
    if(springs[s].pm_a->pinned && springs[s].pm_b->pinned){
      continue;
    } else{

      
      Vector3D displacement = springs[s].pm_b->position-springs[s].pm_a->position;
      if (displacement.norm() > springs[s].rest_length * 1.1){
        double excessLength = displacement.norm() - springs[s].rest_length * 1.1;
        displacement.normalize();
        if (!springs[s].pm_a->pinned && !springs[s].pm_b->pinned){
          springs[s].pm_a->position += excessLength*0.5f*displacement;
          springs[s].pm_b->position -= excessLength*0.5f*displacement;
        } else if(!springs[s].pm_a->pinned){
          springs[s].pm_a->position += excessLength*displacement;
        } else if(!springs[s].pm_b->pinned){
          springs[s].pm_b->position -= excessLength*displacement;
        }
      }
    }
  }

  return;

}

void Cloth::build_spatial_map() {

  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  
  for (int p = 0; p < point_masses.size(); p++){
    float uniqueID = hash_position(point_masses[p].position);
      PointMass *massPtr = &point_masses[p];
      vector<PointMass *> *bucket;
      try{
       bucket = map.at(uniqueID);
      }catch(exception e){
        // cout<< "Out of range, creating bucket" << endl;
        bucket = new vector<PointMass *>();
      }
      PointMass *currentMass = &point_masses[p];
      bucket->emplace_back(currentMass);
      map[uniqueID] = bucket;
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
   float uniqueID = hash_position(pm.position);
   Vector3D correctionVector = Vector3D(0);
   int correctionCount = 0;
  //  cout<< "getting bucket for current pointmass" << endl;
   vector<PointMass*> *potential;
   bool bucketFound = true;
   try{
    potential = map[uniqueID];
   }catch(exception){
     bucketFound = false;
     cout << "no bucket found" << endl;
   }
   if (bucketFound){
    for (int p = 0; p < potential->size(); p++){
      PointMass *candidate = potential->at(p);
      if (candidate != &pm){
        Vector3D correctionDir = candidate->position - pm.position;
        if (correctionDir.norm() < 2 * thickness){
          correctionCount++;
          double correctionDist = correctionDir.norm()-(2*thickness);
          correctionDir.normalize();
          correctionVector += correctionDir*correctionDist;
          
        }
      }
    }

    if (correctionCount != 0){
      pm.position += correctionVector / correctionCount / simulation_steps;
    }
   }

   

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double w = 3 * width / num_width_points;
  double h = 3 * height / num_height_points;
  double t = max(w, h);

  double x = w*floor(pos.x/w);
  double y = h*floor(pos.y/h);
  double z = t*floor(pos.z/t);



  return x*19 + y*23 + z*59;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
