/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/
#include "jello.h"
#include "physics.h"

//Define the lengths of the restsprings
#define REST_LENGTH_STRUCTURAL 1/7.0
#define REST_LENGTH_SHEAR sqrt(pow(1/7.0,2)+pow(1/7.0,2))
#define REST_LENGTH_BEND (1/7.0)*2

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

				}
			}
		}
	}
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
		ComputeForceForTwoPoints(jello, original, neighbor, a, false, REST_LENGTH_STRUCTURAL);
	}

	//Check that we're not the edge case in y
	if(j<7)
	{
		//We're looking at the y neighbor, therefor increment y by 1
		neighbor.xIndex = i;
		neighbor.yIndex = j+1;
		neighbor.zIndex = k;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPoints(jello, original, neighbor, a, false, REST_LENGTH_STRUCTURAL);
	}

	//Check that we're not the edge case in z
	if(k<7)
	{
		//We're looking at the z neighbor, therefor increment z by 1
		neighbor.xIndex = i;
		neighbor.yIndex = j;
		neighbor.zIndex = k+1;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPoints(jello, original, neighbor, a, false, REST_LENGTH_STRUCTURAL);
	}
}

//This function computes the spring forces for two given neighbors
void ComputeForceForTwoPoints(struct world * jello, indicies orginal, indicies neighbor, struct point a[8][8][8], bool isCollision, double restLength)
{
		//Variable for storing the distance between two points.
		point lVector;
		//Variable for storing the total force
		point structuralSpringForce = point();
		pINIT(structuralSpringForce);

		//Variable for storing the hook force
		point hookForce = point();
		pINIT(hookForce);

		//Variable for storing the dampening force
		point dampeningForce = point();
		pINIT(dampeningForce);

		//if we arent, compute the distance and vector to the next x neighbour
		pDIFFERENCE(jello->p[neighbor.xIndex][neighbor.yIndex][neighbor.zIndex], jello->p[orginal.xIndex][orginal.yIndex][orginal.zIndex], lVector);

		//Call the customly written Hook Force Function to get the Hook force
		ComputeHookForce(jello, lVector, restLength, hookForce, isCollision);

		//Call the customly written Dampening Force Function to get the Dampening force
		ComputeDampeningForce(jello, lVector, orginal, neighbor, restLength, dampeningForce, isCollision);

		//Add the dampening and hook forces together
		pSUM(dampeningForce, hookForce, structuralSpringForce);

		//Add the force to the force on vertex [i][j][k]
		pSUM(a[orginal.xIndex][orginal.yIndex][orginal.zIndex], structuralSpringForce, a[orginal.xIndex][orginal.yIndex][orginal.zIndex]);

		//Subtract the force from the force on vertex [i+1][j][k]
		pDIFFERENCE(a[neighbor.xIndex][neighbor.yIndex][neighbor.zIndex], structuralSpringForce, a[neighbor.xIndex][neighbor.yIndex][neighbor.zIndex]);
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
		ComputeForceForTwoPoints(jello, original, neighbor, a, false, REST_LENGTH_BEND);
	}

	//Check that we're not the edge case in y
	if(j<6)
	{
		//We're looking at the y neighbor, therefor increment y by 2
		neighbor.xIndex = i;
		neighbor.yIndex = j+2;
		neighbor.zIndex = k;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPoints(jello, original, neighbor, a, false, REST_LENGTH_BEND);
	}

	//Check that we're not the edge case in z
	if(k<6)
	{
		//We're looking at the z neighbor, therefor increment z by 2
		neighbor.xIndex = i;
		neighbor.yIndex = j;
		neighbor.zIndex = k+2;

		//Compute the structural spring forces for these two neighbors
		ComputeForceForTwoPoints(jello, original, neighbor, a, false, REST_LENGTH_BEND);
	}
}


//This function computes the hook force for two given neighbors
void ComputeHookForce(struct world * jello, point l, double restLength, point returnedForce, bool isCollision)
{
	//This line is required to normalize l
	double length = 0;
	pNORMALIZE(l);

	//Check to see if we're dealing with a collision spring for not.
	if(isCollision)
	{
		//We multiply by the hook coefficient and the difference between actuall length and restlength
		pMULTIPLY(l, -jello->kCollision * (length - restLength), returnedForce);
	}
	else
	{
		//We multiply by the hook coefficient and the difference between actuall length and restlength
		pMULTIPLY(l, -jello->kElastic * (length - restLength), returnedForce);
	}
}

//This function computes the dampening force for two given neighbors
void ComputeDampeningForce(struct world * jello, point l, indicies a, indicies b, double restLength, point returnedForce, bool isCollision)
{
	//This line is required to normalize l
	double length = 0;
	pNORMALIZE(l);

	//find and store the velocities
	point va = jello->v[a.xIndex][a.yIndex][a.zIndex];
	point vb = jello->v[b.xIndex][b.yIndex][b.zIndex];

	//We now find va - vb
	point difference;
	pDIFFERENCE(vb, va, difference);

	//We do the dot product between difference and l
	double dotProduct = 0;
	DOTPRODUCTp(difference, l, dotProduct);

	//Check to see if we're dealing with a collision spring for not.
	if(isCollision)
	{
		//We multiply by the hook coefficient and the difference between actuall length and restlength
		pMULTIPLY(l, -jello->dCollision * dotProduct, returnedForce);
	}
	else
	{
		//We multiply by the hook coefficient and the difference between actuall length and restlength
		pMULTIPLY(l, -jello->dElastic * dotProduct, returnedForce);
	}
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
