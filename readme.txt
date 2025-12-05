1) [2] Correct and automatic initialization of the boids. All boids should be randomly initialized within the initialization radius with a random forward heading within the forward random range.
	Done: all boids are initialized with a random position around transform.position using Random.insideUnitSphere (multiplied by the initialization radius), and a random forward heading by creating a quaternion with a Euler angles in the range (random for x, y, and z); BBoid objects are created and prefabs are instantiated with these random values

2) [3] Correct handling of the resetting of values at the top of the simulation loop
	Done: ResetBoidForces() is called at the beginning of FixedUpdate(), which calculates the forces on each boid, starting from zero (I.e. starting at zero force and adding in the forces rule by rule)

3) [3] Correct implementation of building the neighbour list, by simulating vision using distance and dot product.
	Done: the calculateNeighbors() helper function takes in an index of a boid and for each other boid in the system, checks if a) the other boid is within the right distance (by testing if the sqrMagnitude of the difference vector between their positions <= sqrNeighborDistance), and if it at the right angle to be seen (by checking the dot product of the forward vector of the current boid and the difference vector between the two)

4) [3] Correct implementation of separation rule
	Done: temp = 1/N(sum_{n=0}^N x-x_n); 
	force += sW*(temp.normalized*boidForceScale - v)

5) [3] Correct implementation of alignment rule
	Done: temp = (1/N)(sum_{n=0}^N v_{a_n}); (the -v is part of the force calculation)
	force += aW*(temp.normalized*boidForceScale - v)

6) [3] Correct implementation of cohesion rule
	Done: temp = (1/N)(sum_{n=0}^N x_n)-x;
	force += cW**(temp.normalized*boidForceScale - v)

7) [3] Correct implementation of the no neighbour wander rule
	Done: if neighbours.Count == 0, the three forces above are *not* calculated, and instead the force is wW*(v.normalized*boidForceScale - v)

8) [3] Correct implementation of obstacles rule
	Done: first, obstacles are found using Physics.OverlapSphere(). For each obstacle found, the vector (x - o.ClosestPointOnBounds(x)).normalized is added to temp. Additionally, the walls are checked manually and their normals are added as well; after all obstacles (including walls) are accounted for, the force oW*(temp.normalized*boidForceScale - v) is added to the current forces

9) [3] Correct addition of the world boundary to the obstacles rule
	Done: the world axis's are checked for X-axis -8, 8; Z-axis -8, 8; Y-axis 1, 4 using boids[I].position.(x, y, or z); if the void is found out of bounds, a normal force is adding pointing back into bounds (like the example x > 8f --> normal=(-1, 0, 0) is added)

10) [5] The total forces are accumulated correctly
	Done: the forces are totaled using w_k(rule.normalized*alpha - v) for each rule

11) [3] SetGoal is implemented correctly, implementing the behaviour described above. A target is set, path calculated.
	Done: when SetGoal is called, the code looks for the point on the NavMesh nearest to goal using SamplePosition(), and then looks for the point on the NavMesh closest to the current point of boidZero. If both these points are found, it creates the path using CalculatePath()

12) [5] boidzero is handled correctly. A check is made to ensure we should be processing the path (navigating + ready + enough corners). Position is correctly sampled from the navmesh. Bookkeeping for corners is correct and all corners are followed. Finishing the path is handled cleanly and variables are reset as needed.
	Done: if a) boidZeroNavigatingTowardGoal is set to true, b) the status of the path is complete, and c) the number of corners in the path is more than 1, then boidZero attempts to follow the path. It does this by first checking what point on the NavMesh boidZero is currently nearest to. If this point is within 1f of the current corner the void is moving to, it begins moving towards the next one (unless that was the last corner in the list in which case boidZeroNavigatingTowardGoal is set to false, currentCorner is reset to 0, and ClearCorners() is called on the path. After this, gW*(toCurrentGoal.normalized*boidForceScale - v) is added to the force, where toCurrentCorner = boidZeroPath.corners[currentCorner]-currentBoidNavMesh

13) [2] The boid objects (animated meshes) are update correctly and follow the path and heading of the boid particles.
	Done: When the position, velocity, and forward vectors of the BBoids are updated, so are the position and euler angles of the boid objects, so the objects are correctly in sync with their BBoids counterparts.

14) [2] The symplectic Euler integration scheme is implemented correctly.
	Done: the new velocities and positions for the boids are calculated using the symplectic Euler integration, which calculates the velocity using the previous velocity and current forces and then calculates the position using the previous position and *new* velocity. (Same as in a2)

15) [3] The simulator loop correctly updates all boid states using the correct update callback and time. Recall the lesson on simulator loops.
	Done: the boids are updated in FixedUpdate() using Time.fixedDeltaTime (same as a2)

16) [1] The recorded testcase requires that the default values in the testcase are preserved.
	It is hard to tell due to the randomness but I believe my boids act in a similar manner

17) You must submit a SINGLE file called <lastname>-<firstname>-a3.zip (replace the <> with your information) that includes all the necessary files. You must follow the .gitignore pattern for Unity. [-2 marks if you do not follow these guidelines]
	Done: this folder

18) You must include a readme.txt file that describes in full detail which of the required elements you have completed successfully and which ones you have not. [-5 Marks if you do not include this file, and partial loss of marks for each component you do not include].
	Done: this file
