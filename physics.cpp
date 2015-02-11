/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/
#include "jello.h"
#include "physics.h"

//Define the lengths of the restsprings
#define REST_LENGTH_STRUCTURAL (1.0f/7.0f)
#define REST_LENGTH_SHEAR_SHORT (sqrt(2.0f)/7.0f)
#define REST_LENGTH_SHEAR_LONG (sqrt(3.0f)/7.0f)
#define REST_LENGTH_BEND (1.0f/7.0f)*2.0f
#define MIN_X_COLLISION -2
#define MIN_Y_COLLISION -2
#define MIN_Z_COLLISION -2
#define MAX_X_COLLISION 2
#define MAX_Y_COLLISION 2
#define MAX_Z_COLLISION 2

int structuralSprings = 0;
int shearSprings = 0;
int shearSpringsLong = 0;
int bendSprings = 0;

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  /*You will have to implement the acceleration function (computeAcceleration, see physics.cpp). 
	This function takes as input the positions of the 512 nodes, the velocities of the 512 nodes, 
	plus the physical parameters of the model. It returns the acceleration for each of the 512 points. 
	It also adds any effects of an external force field, if such a field is present of course. 
	In general, the acceleration will of course be different for each of the 512 simulation points. 
	To compute the acceleration, the function must take into account the forces due to:

		structural, shear and bend springs,
		external forces (force field, if any), and
		bouncing off the walls. */

	//initialize a first of all
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			for (int k = 0; k < 8; k++)
			{
				pINIT(a[i][j][k]);
			}
		}
	}

	//We need to go through all points, therefore we need a triple nested for loop:
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			for (int k = 0; k < 8; k++)
			{
				/*We compute the forces of the springs given one spring type at a time.
				We also start from the [0][0][0] and progressively check the neighbours, improving the efficiency
				So for structural [0][0][0] checks and computes forces for [0][0][1], [1][0][0] and [0][1][0] and
				These wont have to check [0][0][0] when it's their turn.*/

				//Structural springs
				{
					ComputeStructuralSpringForceMain(jello, i, j, k, a);
				}

				//Shear springs
				{
					ComputeShearSpringForceMain(jello, i, j, k, a);
				}

				//Bend springs
				{
					ComputeBendSpringForceMain(jello, i, j, k, a);
				}

				//Force Field
				{
					ComputeForceFieldAcceleration(jello,i , j, k, a);
				}

				//Collision Detection
				{
					CollionDetectionMain(jello, i, j, k, a);
				}

				//In order to ensure the cube doesnt go crazy at high speeds, 
				//we reduce the speed if it's higher than a cetain threshold
				//Make sure the velocity doesnt spin out of hand
				double length = 0;
				pLENGTH(jello->v[i][j][k], length);
				if(length > 50.0f)
				{
					pMULTIPLY(jello->v[i][j][k], 0.75f, jello->v[i][j][k]);
				}
			}
		}
	}

	structuralSprings += 0;
	shearSprings += 0;
	bendSprings += 0;
	shearSpringsLong += 0;
}

//This function find the acceleration of the forcefield that should be added to the acceleration
void ComputeForceFieldAcceleration(struct world * jello, int i, int j, int k, struct point a[8][8][8])
{
	//Save the resolution as n
	int n = jello->resolution-1;

	//We interpolate the x, y and z's to be a value between 0 and 1
	float polX = (jello->p[i][j][k].x + 2.0f) / 4.0f;
	float polY = (jello->p[i][j][k].y + 2.0f) / 4.0f;
	float polZ = (jello->p[i][j][k].z + 2.0f) / 4.0f;

	//Now, we find the x, y and z min and max
	int minX = floor(abs(polX) * n);
	int maxX = minX + 1;
	int minY = floor(abs(polY) * n);
	int maxY = minY +1 ;
	int minZ = floor(abs(polZ) * n);
	int maxZ = minZ + 1;

	if((maxX * n *n + maxY * n + maxZ) > (29*29*29+29*29+29))
		return;

	//We store the 8 forces of the points of the box around our point
	//I use capitalization to distinguish between min and max of a value
	//We access the point based on the implemented in createWorld.cpp
	point xyz = jello->forceField[minX * n * n + minY * n + minZ]; 
	point xyZ = jello->forceField[minX * n * n + minY * n + maxZ]; 
	point xYz = jello->forceField[minX * n * n + maxY * n + minZ]; 
	point xYZ = jello->forceField[minX * n * n + maxY * n + maxZ]; 
	point Xyz = jello->forceField[maxX * n * n + minY * n + minZ]; 
	point XyZ = jello->forceField[maxX * n * n + minY * n + maxZ]; 
	point XYz = jello->forceField[maxX * n * n + maxY * n + minZ]; 
	point XYZ = jello->forceField[maxX * n * n + maxY * n + maxZ]; 

	//Compute the force with the very very very long equation from class
	//Multiply all the scalars with all the forces
	pMULTIPLY(xyz, (1-polX)*(1-polY)*(1-polZ), xyz);
	pMULTIPLY(xyZ, (1-polX)*(1-polY)*(polZ), xyZ);
	pMULTIPLY(xYz, (1-polX)*(polY)*(1-polZ), xYz);
	pMULTIPLY(xYZ, (1-polX)*(polY)*(polZ), xYZ);
	pMULTIPLY(Xyz, (polX)*(1-polY)*(1-polZ), Xyz);
	pMULTIPLY(XyZ, (polX)*(1-polY)*(polZ), XyZ);
	pMULTIPLY(XYz, (polX)*(polY)*(1-polZ), XYz);
	pMULTIPLY(XYZ, (polX)*(polY)*(polZ), XYZ);

	//Divide by the mass for all forces, so we have F/m = a
	pDIVIDE(xyz, jello->mass, xyz);
	pDIVIDE(xyZ, jello->mass, xyZ);
	pDIVIDE(xYz, jello->mass, xYz);
	pDIVIDE(xYZ, jello->mass, xYZ);
	pDIVIDE(Xyz, jello->mass, Xyz);
	pDIVIDE(XyZ, jello->mass, XyZ);
	pDIVIDE(XYz, jello->mass, XYz);
	pDIVIDE(XYZ, jello->mass, XYZ);

	//Add the forces to the acceleration
	pSUM(a[i][j][k], xyz, a[i][j][k]);
	pSUM(a[i][j][k], xyZ, a[i][j][k]);
	pSUM(a[i][j][k], xYz, a[i][j][k]);
	pSUM(a[i][j][k], xYZ, a[i][j][k]);
	pSUM(a[i][j][k], Xyz, a[i][j][k]);
	pSUM(a[i][j][k], XyZ, a[i][j][k]);
	pSUM(a[i][j][k], XYz, a[i][j][k]);
	pSUM(a[i][j][k], XYZ, a[i][j][k]);
}

//This function computes the structural spring forces for the given point
void ComputeStructuralSpringForceMain(struct world * jello, int i, int j, int k, struct point a[8][8][8])
{
	//Variable for storing the indicies of the points we're looking at
	indicies original = indicies();
	indicies neighbor = indicies();

	//This will be the same throughout, so store it now
	original.xIndex = i;
	original.yIndex = j;
	original.zIndex = k;

	//Check that we're not the edge case in x
	if(i<7)
	{
		//We're looking at the x neighbor, therefor increment x by 1
		neighbor.xIndex = i+1;
		neighbor.yIndex = j;
		neighbor.zIndex = k;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_STRUCTURAL);
		structuralSprings += 1;
	}

	//Check that we're not the edge case in y
	if(j<7)
	{
		//We're looking at the y neighbor, therefor increment y by 1
		neighbor.xIndex = i;
		neighbor.yIndex = j+1;
		neighbor.zIndex = k;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_STRUCTURAL);
		structuralSprings += 1;
	}

	//Check that we're not the edge case in z
	if(k<7)
	{
		//We're looking at the z neighbor, therefor increment z by 1
		neighbor.xIndex = i;
		neighbor.yIndex = j;
		neighbor.zIndex = k+1;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_STRUCTURAL);
		structuralSprings += 1;
	}
}

//This function computes the shear spring forces for the given point
void ComputeShearSpringForceMain(struct world * jello, int i, int j, int k, struct point a[8][8][8])
{
	//Variable for storing the indicies of the points we're looking at
	indicies original = indicies();
	indicies neighbor = indicies();

	//This will be the same throughout, so store it now
	original.xIndex = i;
	original.yIndex = j;
	original.zIndex = k;

	//Check that we can reach the vertex straight across from us
	if(i<7 && j<7)
	{
		//We're looking at the x neighbor, therefor increment x by 1
		neighbor.xIndex = i+1;
		neighbor.yIndex = j+1;
		neighbor.zIndex = k;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_SHORT);
		shearSprings++;
	}

	//Check that we can reach the ´vertex straight across to the right of us
	if(i<7 && j>0)
	{
		//We're looking at the y neighbor, therefor increment y by 1
		neighbor.xIndex = i+1;
		neighbor.yIndex = j-1;
		neighbor.zIndex = k;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_SHORT);
		shearSprings++;
	}

	//Check that we can reach the vertex down to the left
	if(k>0 && j<7)
	{
		//We're looking at the z neighbor, therefor increment z by 1
		neighbor.xIndex = i;
		neighbor.yIndex = j+1;
		neighbor.zIndex = k-1;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_SHORT);
		shearSprings++;
	}

	//Check that we can reach the vertex down to the right
	if(k>0 && j>0)
	{
		//We're looking at the z neighbor, therefor increment z by 1
		neighbor.xIndex = i;
		neighbor.yIndex = j-1;
		neighbor.zIndex = k-1;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_SHORT);
		shearSprings++;
	}

	//Check that we can reach the vertex down and back
	if(i>0 && k>0)
	{
		//We're looking at the z neighbor, therefor increment z by 1
		neighbor.xIndex = i-1;
		neighbor.yIndex = j;
		neighbor.zIndex = k-1;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_SHORT);
		shearSprings++;
	}

	//Check that we can reach the vertex down and straight
	if(i<7 && k>0)
	{
		//We're looking at the z neighbor, therefor increment z by 1
		neighbor.xIndex = i+1;
		neighbor.yIndex = j;
		neighbor.zIndex = k-1;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_SHORT);
		shearSprings++;
	}
	//Compute the straight diagonals
	if(k < 7)
	{
		//Check down straight left
		if(i < 7 && j <7)
		{
			//We're looking at the z neighbor, therefor increment z by 1
			neighbor.xIndex = i+1;
			neighbor.yIndex = j+1;
			neighbor.zIndex = k+1;

			//Compute the structural spring forces for these two neighbors
			ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_LONG);
			shearSpringsLong++;
		}

		//Check down back left
		if(i > 0 && j <7)
		{
			//We're looking at the z neighbor, therefor increment z by 1
			neighbor.xIndex = i-1;
			neighbor.yIndex = j+1;
			neighbor.zIndex = k+1;

			//Compute the structural spring forces for these two neighbors
			ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_LONG);
			shearSpringsLong++;
		}

		//Check down back right
		if(i > 0 && j > 0)
		{
			//We're looking at the z neighbor, therefor increment z by 1
			neighbor.xIndex = i-1;
			neighbor.yIndex = j-1;
			neighbor.zIndex = k+1;

			//Compute the structural spring forces for these two neighbors
			ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_LONG);
			shearSpringsLong++;
		}

		//Check down straight right
		if(i < 7 && j > 0)
		{
			//We're looking at the z neighbor, therefor increment z by 1
			neighbor.xIndex = i+1;
			neighbor.yIndex = j-1;
			neighbor.zIndex = k+1;

			//Compute the structural spring forces for these two neighbors
			ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_SHEAR_LONG);
			shearSpringsLong++;
		}
	}
}

//This function computes the bend spring forces for the 
void ComputeBendSpringForceMain(struct world * jello, int i, int j, int k, struct point a[8][8][8])
{
	//Variable for storing the indicies of the points we're looking at
	indicies original = indicies();
	indicies neighbor = indicies();

	//This will be the same throughout, so store it now
	original.xIndex = i;
	original.yIndex = j;
	original.zIndex = k;

	//Check that we're not the edge case in x
	if(i<6)
	{
		//We're looking at the x neighbor, therefor increment x by 2
		neighbor.xIndex = i+2;
		neighbor.yIndex = j;
		neighbor.zIndex = k;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_BEND);
		bendSprings++;

	}

	//Check that we're not the edge case in y
	if(j<6)
	{
		//We're looking at the y neighbor, therefor increment y by 2
		neighbor.xIndex = i;
		neighbor.yIndex = j+2;
		neighbor.zIndex = k;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_BEND);
		bendSprings++;
	}

	//Check that we're not the edge case in z
	if(k<6)
	{
		//We're looking at the z neighbor, therefor increment z by 2
		neighbor.xIndex = i;
		neighbor.yIndex = j;
		neighbor.zIndex = k+2;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPointsElastic(jello, original, neighbor, a, REST_LENGTH_BEND);
		bendSprings++;
	}
}

//This function computes the spring forces for two given neighbors
void ComputeForceForTwoPointsElastic(struct world * jello, indicies orginal, indicies neighbor, struct point a[8][8][8], double restLength)
{
		//Variable for storing the distance between two points.
		point lVector;
		pINIT(lVector);

		//Variable for storing the total force
		point totalSpringForce = point();
		pINIT(totalSpringForce);

		//Variable for storing the hook force
		point hookForce = point();
		pINIT(hookForce);

		//Variable for storing the dampening force
		point dampeningForce = point();
		pINIT(dampeningForce);

		//Compute the distance and vector to the next x neighbour
		pDIFFERENCE(jello->p[orginal.xIndex][orginal.yIndex][orginal.zIndex], jello->p[neighbor.xIndex][neighbor.yIndex][neighbor.zIndex], lVector);

		//Call the customly written Hook Force Function to get the Hook force
		ComputeHookForce(jello, lVector, restLength, &hookForce, false);

		//find and store the velocities to use when computing the dampening forces
		point va = jello->v[orginal.xIndex][orginal.yIndex][orginal.zIndex];
		point vb = jello->v[neighbor.xIndex][neighbor.yIndex][neighbor.zIndex];

		//Call the customly written Dampening Force Function to get the Dampening force
		ComputeDampeningForce(jello, lVector, va, vb, restLength, &dampeningForce, false);

		//Add the dampening and hook forces together
		pSUM(dampeningForce, hookForce, totalSpringForce);

		//Divide by the mass, so that we have F/m = a
		pDIVIDE(totalSpringForce, jello->mass, totalSpringForce);

		//add the force to the force on vertex [i][j][k]
		pSUM(a[orginal.xIndex][orginal.yIndex][orginal.zIndex], totalSpringForce, a[orginal.xIndex][orginal.yIndex][orginal.zIndex]);

		//Subtract the force from the force on neighboring vertex
		pDIFFERENCE(a[neighbor.xIndex][neighbor.yIndex][neighbor.zIndex], totalSpringForce, a[neighbor.xIndex][neighbor.yIndex][neighbor.zIndex]);
}

//This function computes the spring forces for a potential collision
void ComputeForceForCollision(struct world * jello, point vertexPoint, indicies vertexindicies, point collisionPoint, struct point a[8][8][8])
{
		//Variable for storing the distance between two points.
		point lVector;
		pINIT(lVector);

		//Variable for storing the total force
		point totalForce = point();
		pINIT(totalForce);

		//Variable for storing the hook force
		point hookForce = point();
		pINIT(hookForce);

		//Variable for storing the dampening force
		point dampeningForce = point();
		pINIT(dampeningForce);

		//Compute the distance and vector to the next x neighbour
		pDIFFERENCE(collisionPoint, vertexPoint, lVector);

		//Call the customly written Hook Force Function to get the Hook force
		//Rest length is 0 since we're dealing with Collision forces
		ComputeHookForce(jello, lVector, 0, &hookForce, true);

		//find and store the velocities to use when computing the dampening forces
		//Note that the velocity of the collision point is simply 0,0,0
		point va = jello->v[vertexindicies.xIndex][vertexindicies.yIndex][vertexindicies.zIndex];
		point vb;
		pINIT(vb);

		//Call the customly written Dampening Force Function to get the Dampening force
		ComputeDampeningForce(jello, lVector, va, vb, 0, &dampeningForce, true);

		//Add the dampening and hook forces together
		pSUM(dampeningForce, hookForce, totalForce);

		//Divide by the mass, so that we have F/m = a
		pDIVIDE(totalForce, jello->mass, totalForce);

		//Substract the force to the force on vertex [i][j][k]
		pDIFFERENCE(a[vertexindicies.xIndex][vertexindicies.yIndex][vertexindicies.zIndex], totalForce, a[vertexindicies.xIndex][vertexindicies.yIndex][vertexindicies.zIndex]);
}

//This function computes the hook force for two given neighbors
void ComputeHookForce(struct world * jello, point l, double restLength, point * returnedForce, bool isCollision)
{
	//This line is required to normalize l
	double length = 0.0f;
	pNORMALIZE(l);

	//Check to see if we're dealing with a collision spring for not.
	if(isCollision)
	{
		//We multiply by the hook coefficient and the difference between actuall length and restlength
		pMULTIPLY(l, -jello->kCollision * (length - restLength), *returnedForce);
	}
	else
	{
		//We multiply by the hook coefficient and the difference between actuall length and restlength
		pMULTIPLY(l, -jello->kElastic * (length - restLength), *returnedForce);
	}
}

//This function computes the dampening force for two given neighbors
void ComputeDampeningForce(struct world * jello, point l, point va, point vb, double restLength, point * returnedForce, bool isCollision)
{
	//Getting the length of the l vector
	double length = 0;
	pLENGTH(l, length);

	//We now find va - vb
	point difference;
	pDIFFERENCE(va, vb, difference);

	//We do the dot product between difference and l
	double dotProduct = 0;
	DOTPRODUCTp(difference, l, dotProduct);

	// We need to normalize l for the formula
	pNORMALIZE(l);

	//Check to see if we're dealing with a collision spring for not.
	if(isCollision)
	{
		//We multiply by the hook coefficient and the difference between actual length and restlength
		pMULTIPLY(l, -jello->dCollision * (dotProduct/length), *returnedForce);
	}
	else
	{
		//We multiply by the hook coefficient and the difference between actual length and restlength
		pMULTIPLY(l, -jello->dElastic * (dotProduct/length), *returnedForce);
	}
}

//Collision detections main function. This is always called to check Collision
void CollionDetectionMain(struct world * jello, int i, int j, int k, struct point a[8][8][8])
{
	//Save the position of the box we're currently looking at.
	point positionOfVertex = jello->p[i][j][k];

	//Use a variable to store whether or not we should compute the collision force or not
	bool didCollide = false;

	//Store the collision point as the original position at first.
	point collisionPoint;

	pCPY(positionOfVertex, collisionPoint);

	//initialize the indicies of the point we're looking at:
	indicies vertexIndicies;
	vertexIndicies.xIndex = i;
	vertexIndicies.yIndex = j;
	vertexIndicies.zIndex = k;

	//Check collision on the x axis. set didCollide to true if we actually did collicde with something
	if(MIN_X_COLLISION > positionOfVertex.x)
	{
		collisionPoint.x = MIN_X_COLLISION;
		didCollide = true;
	}
	else if(MAX_X_COLLISION < positionOfVertex.x)
	{
		collisionPoint.x = MAX_X_COLLISION;
		didCollide = true;
	}

	//Check collision on the y axis
	if(MIN_Y_COLLISION > positionOfVertex.y)
	{
		collisionPoint.y = MIN_Y_COLLISION;
		didCollide = true;
	}
	else if(MAX_Y_COLLISION < positionOfVertex.y)
	{
		collisionPoint.y = MAX_Y_COLLISION;
		didCollide = true;
	}

	//Check collision on the z axis
	if(MIN_Z_COLLISION > positionOfVertex.z)
	{
		collisionPoint.z = MIN_Z_COLLISION;
		didCollide = true;
	}
	else if(MAX_Z_COLLISION < positionOfVertex.z)
	{
		collisionPoint.z = MAX_Z_COLLISION;
		didCollide = true;
	}

	//Only compute the collision force, if we actually collided with something
	if(didCollide)
		ComputeForceForCollision(jello, positionOfVertex, vertexIndicies, collisionPoint, a);
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
