1) [2] Correct and automatic initialization of the boids. All boids should be randomly initialized within the initialization radius with a random forward heading within the forward random range.

2) [3] Correct handling of the resetting of values at the top of the simulation loop
[3] Correct implementation of building the neighbour list, by simulating vision using distance and dot product.

3) [3] Correct implementation of separation rule

4) [3] Correct implementation of alignment rule

5) [3] Correct implementation of cohesion rule

6) [3] Correct implementation of the no neighbour wander rule

7) [3] Correct implementation of obstacles rule

8) [3] Correct addition of the world boundary to the obstacles rule

9) [5] The total forces are accumulated correctly

10) [3] SetGoal is implemented correctly, implementing the behaviour described above. A target is set, path calculated.

11) [5] boidzero is handled correctly. A check is made to ensure we should be processing the path (navigating + ready + enough corners). Position is correctly sampled from the navmesh. Bookkeeping for corners is correct and all corners are followed. Finishing the path is handled cleanly and variables are reset as needed.

12) [2] The boid objects (animated meshes) are update correctly and follow the path and heading of the boid particles.

13) [2] The symplectic Euler integration scheme is implemented correctly.

14) [3] The simulator loop correctly updates all boid states using the correct update callback and time. Recall the lesson on simulator loops.

15) [1] The recorded testcase requires that the default values in the testcase are preserved.

16) You must submit a SINGLE file called <lastname>-<firstname>-a3.zip (replace the <> with your information) that includes all the necessary files. You must follow the .gitignore pattern for Unity. [-2 marks if you do not follow these guidelines]

17) You must include a readme.txt file that describes in full detail which of the required elements you have completed successfully and which ones you have not. [-5 Marks if you do not include this file, and partial loss of marks for each component you do not include].
	Done: this file
