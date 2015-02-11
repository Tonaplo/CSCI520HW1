/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

struct indicies 
{
   int xIndex;
   int yIndex;
   int zIndex;
};

void computeAcceleration(struct world * jello, struct point a[8][8][8]);

//My functions
//Structural Forces
void ComputeStructuralSpringForceMain(struct world * jello, int i, int j, int k, struct point a[8][8][8]);

//Shear Forces
void ComputeShearSpringForceMain(struct world * jello, int i, int j, int k, struct point a[8][8][8]);

//Bend Forces
void ComputeBendSpringForceMain(struct world * jello, int i, int j, int k, struct point a[8][8][8]);

//General Forces
void ComputeForceForTwoPointsElastic(struct world * jello, indicies orginal, indicies neighbor, struct point a[8][8][8], double restLength);
void ComputeForceForCollision(struct world * jello, point vertexPoint, indicies vertexindicies, point collisionPoint, struct point a[8][8][8]);
void ComputeHookForce(struct world * jello, point l, double restLength, point * returnedForce, bool isCollision);
void ComputeDampeningForce(struct world * jello, point l, point va, point vb, double restLength, point * returnedForce, bool isCollision);

//Collision Detection
void CollionDetectionMain(struct world * jello, int i, int j, int k, struct point a[8][8][8]);

//Forcefield interpolation
void ComputeForceFieldAcceleration(struct world * jello, int i, int j, int k, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

#endif

