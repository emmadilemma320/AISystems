using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;

    public int numberOfBoids = 200;

    public float boidForceScale = 20f;

    public float maxSpeed = 5.0f;

    public float rotationSpeed = 40.0f;

    public float obstacleCheckRadius = 1.0f;

    public float separationWeight = 1.1f;
    
    public float alignmentWeight = 0.5f;

    public float cohesionWeight = 1f;

    public float goalWeight = 1f;

    public float obstacleWeight = 0.9f;

    public float wanderWeight = 0.3f;

    public float neighbourDistance = 2.0f; // fov = 180

    public float initializationRadius = 1.0f;

    public float initializationForwardRandomRange = 50f;

    private BBoid[] boids;

    private Transform[] boidObjects;

    private float sqrNeighbourDistance; 

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;

    private Vector2[] worldBounds = {new Vector2(-8f, 8f), new Vector2(1f, 4f), new Vector2(-8f, 8f)}; //  {x, y, z}
    private Vector3[] new_positions;
    private Vector3[] new_velocitys;

    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = Mathf.Sqrt(neighbourDistance);
        InitBoids();
        new_positions = new Vector3[numberOfBoids];
        new_velocitys = new Vector3[numberOfBoids];

    }

    /// <summary>
    /// Initialize the array of boids
    /// </summary>
    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
       for(int i = 0; i < numberOfBoids; i++){
            BBoid new_boid= new BBoid();
            // set parameters (velocity and forces are calculated later)
            new_boid.position = new Vector3(); // random
            new_boid.forward = new Vector3(); // random
            // add it to boids
            boids[i] = new_boid;

            // intialize an instance of the boidPrefab
            Transform new_bug = Instantiate(boidPrefab);
            new_bug.position = boids[i].position;
            new_bug.parent = this.transform;
       }
    }


    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        for(int i = 0; i < numberOfBoids; i++){
            int[] neighbors = calculateNeighbors(i);
            boids[i].currentTotalForce = Vector3.zero;
            if(neighbors.Length != 0){
                // alignment = (1/N)(sum_{n=0}^N v_{a_n}) - v

                boids[i].currentTotalForce += alignmentWeight*(boids[i].alignment*boidForceScale - boids[i].velocity);

                // cohesion = (1/N)(sum_{n=0}^N x_n)-x

                boids[i].currentTotalForce += cohesionWeight*(boids[i].cohesion*boidForceScale - boids[i].velocity);

                // separation = 1/N(sum_{n=0}^N x-x_n)

                boids[i].currentTotalForce += separationWeight*(boids[i].separation*boidForceScale - boids[i].velocity);
            } else {
                // wander = v_i
                boids[i].currentTotalForce += wanderWeight*(boids[i].velocity*boidForceScale - boids[i].velocity);
            }

            // obstacle = -
        }

        // then add the path info to boid zero
    }

    private int[] calculateNeighbors(int i){
        int[] neighbors = new int[] {};
        for(int j = 0; j < numberOfBoids; j++){
            if(j != i){
                // distance check 
                if(visible(i, j)) {
                    neighbors[neighbors.Length]=j; // idk if this will add corectly
                }
            }
        }
        return neighbors;
    }

    private bool visible(int i, int j){

        return true;
    }


    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        // first we calculate the boid forces
        ResetBoidForces();

        // next, we calculate the new velocities and positions using Symplectic Euler (same integration scheme as assignment 1)
        for(int i = 0; i < numberOfBoids; i++){
            new_velocitys[i] = boids[i].velocity + Time.fixedDeltaTime*boids[i].currentTotalForce;
            new_positions[i] = boids[i].position + Time.fixedDeltaTime*new_velocitys[i];
        }

        // next we update all new velocities and positions
        for(int i = 0; i < numberOfBoids; i++){
            boids[i].velocity = new_velocitys[i];
            boids[i].position = new_positions[i];
        }
    }


    private void Update()
    {
        /* Render information for boidzero, useful for debugging forces and path planning
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                    Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
            { 
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }
        
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment, Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion, Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle, Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }
        */
    }


    public void SetGoal(Vector3 goal)
    {
        if(!boidZeroNavigatingTowardGoal){
            boidZeroGoal = goal;
            boidZeroNavigatingTowardGoal = true;
        }
    }
}

