<Please submit this file with your solution.>

CSCI 520, Assignment 1

<Your name>

================

<Description of what you have accomplished>

I have implemented all of the required parts of the assignment.

For my own ends and needs, I have extended the API defines with pDIVIDE, pLENGTH, pDOTPRODUCT and pINIT. Furthermore, the commenting on pDIFFERENCE was wrong, so I changed it to be correct.
I did not implement any datastructure for the springs, as I found them unnecessary for the size of the assignment.

Going through the vertices, I have implemented a certain pattern to make sure I never calculate or even evaluate the same spring twice.

The pattern moves forward from every given vertex at (i,j,k). I have explained the pattern to professor Barbic once before.

For the structural springs, I go move forward in i, j and k direction, meaning for (i,j,k), I check (i+1,j,k), (i,j+1,k) and (i,j,k+1).

I do the same for bend springs, except add 2, meaning for (i,j,k) I check (i+2,j,k), (i,j+2,k) and (i,j,k+2).

The shear springs are slightly more complicated. 
I've divided them into two different types - the ones of length sqrt(2), going across a face of a cube and the ones of length sqrt(3) going across inside a cube.

The first type has 3 cases:

	1. moving out and forward along the z plane, (i+1, j+1, k) and (i+1, j-1, k)
	2. moving down and forward/back along the y plane, (i-1, j, k-1) and (i+1, j, k-1)
	3. moving down and sideways along the x plane, (i, j+1, k-1) and (i, j-1, k-1)
	
Basically, this pattern is followed for each plane:

		*  *
		  /
		 /
		*  *
		 \
		  \
		*  *
		
Finally, the sqrt(3) diagonal shear springs simply sticks 4 fours down wards to each side, where the middle vertex is at level k and all others are at k+1:

	 *  *  *
	  \   /
	   \ /
	 *	*  *
	   / \
	  /	  \
	 *	*  *
	 
This means from (i,j,k), we check (i+1,j+1,k-1), (i-1,j+1,k-1), (i+1,j-1,k-1) and (i-1,j-1,k-1)

Summing up the amount of calculation, equals the amount of springs.

Using this pattern isnt the most obvious way of doing it and the code may seem abit obscure, even though I've tried to comment it as best I could.
However, it is very efficient, as it only traverses all the springs once and uses the calculations done for one vertex to whichever other vertex the spring is connected to.


<Also, explain any extra credit that you have implemented.>

