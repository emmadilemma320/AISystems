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

    private Vector3[] new_positions;
    private Vector3[] new_velocitys;

    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
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
        boidObjects = new Transform[numberOfBoids];
       for(int i = 0; i < numberOfBoids; i++){
            BBoid new_boid = new BBoid();
            // set parameters (velocity and forces are calculated later)
            new_boid.position = Random.insideUnitSphere*initializationRadius + transform.position; 
            Quaternion q = new Quaternion();
                float x = Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange);
                float y = Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange);
                float z = Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange);
            q.eulerAngles = new Vector3(x, y, z);
            new_boid.forward = q*Vector3.forward; 
            // add it to boids
            boids[i] = new_boid;

            // intialize an instance of the boidPrefab
            Transform new_bug = Instantiate(boidPrefab);
            new_bug.position = boids[i].position;
            new_bug.eulerAngles = boids[i].forward;
            new_bug.parent = this.transform;
            boidObjects[i] = new_bug;
       }
    }


    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        for(int i = 0; i < numberOfBoids; i++){
            List<int> neighbors = calculateNeighbors(i); int N = neighbors.Count;
            boids[i].currentTotalForce = Vector3.zero;
            Vector3 temp;
            Vector3 x = boids[i].position;
            if(N != 0){
                // alignment = (1/N)(sum_{n=0}^N v_{a_n}) - v

                temp = Vector3.zero;
                foreach(int n in neighbors){ // from 0 to N
                    temp += boids[n].velocity; // v_{a_n}
                }
                temp /= N; // 1/N
                //temp -= boids[i].velocity; // -v (may be incorrect)
                boids[i].alignment = temp.normalized; // rule_{ki} should be normalized
                boids[i].currentTotalForce += alignmentWeight*(boids[i].alignment*boidForceScale - boids[i].velocity);


                // cohesion = (1/N)(sum_{n=0}^N x_n)-x

                temp = Vector3.zero;
                foreach(int n in neighbors){ // from 0 to N
                    temp += boids[n].position; // x_n
                }
                temp /= N; // 1/N
                temp -= x; // -x
                boids[i].cohesion = temp.normalized; // rule_{ki} should be normalized
                boids[i].currentTotalForce += cohesionWeight*(boids[i].cohesion*boidForceScale - boids[i].velocity);

                // separation = 1/N(sum_{n=0}^N x-x_n)
                temp = Vector3.zero;
                foreach(int n in neighbors){ // from 0 to N
                    temp = x - boids[n].position; // x - x_n
                }
                temp /= N; // 1/N
                boids[i].separation = temp.normalized; // rule_{ki} should be normalized
                boids[i].currentTotalForce += separationWeight*(boids[i].separation*boidForceScale - boids[i].velocity);
            } else {
                // wander = v_i
                boids[i].currentTotalForce += wanderWeight*(boids[i].velocity*boidForceScale - boids[i].velocity);
            }

            // obstacle = sum of all obstacle normals within the obstacle check radius
            temp = Vector3.zero;
            Collider[] obstacles = Physics.OverlapSphere(boids[i].position, obstacleCheckRadius);
            foreach(Collider o in obstacles){ 
                temp += (x - o.ClosestPointOnBounds(x)).normalized;
            }
            // plus all the walls
            if(x.x > 8f){
                temp += Vector3.left;
            } else if(x.x < -8f){
                temp += Vector3.right;
            }
            if(x.z > 8f){
                temp += Vector3.back;
            } else if(x.z < -8f){
                temp += Vector3.forward;
            }
            if(x.y > 4){
                temp += Vector3.down;
            } else if(x.y < 1){
                temp += Vector3.up;
            }
            boids[i].obstacle = temp.normalized;
            boids[i].currentTotalForce += obstacleWeight*(boids[i].obstacle*boidForceScale - boids[i].velocity);
        }

        // then add the path info to boid zero
    }

    private List<int> calculateNeighbors(int i){
        List<int> neighbors = new List<int>();
        for(int j = 0; j < numberOfBoids; j++){
            if(j != i){
                Vector3 diffV = boids[i].position - boids[j].position;
                if((diffV.sqrMagnitude < sqrNeighbourDistance) && // distance check
                    (Vector3.Dot(boids[i].forward, diffV) > 0)) // FOV check (180)
                {
                    neighbors.Add(j); // idk if this will add corectly
                }
            }
        }
        return neighbors;
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
            if(new_velocitys[i].x > maxSpeed){
                new_velocitys[i].x = maxSpeed;
            } else if(new_velocitys[i].x < -maxSpeed){
                new_velocitys[i].x = -maxSpeed;
            }
            if(new_velocitys[i].y > maxSpeed){
                new_velocitys[i].y = maxSpeed;
            } else if(new_velocitys[i].y < -maxSpeed){
                new_velocitys[i].y = -maxSpeed;
            }
            if(new_velocitys[i].z > maxSpeed){
                new_velocitys[i].z = maxSpeed;
            } else if(new_velocitys[i].z < -maxSpeed){
                new_velocitys[i].z = -maxSpeed;
            }
            
            new_positions[i] = boids[i].position + Time.fixedDeltaTime*new_velocitys[i];

        }

        // next we update all new velocities and positions
        for(int i = 0; i < numberOfBoids; i++){
            boids[i].velocity = new_velocitys[i]; 
            boids[i].forward = boids[i].velocity.normalized;
            boids[i].position = new_positions[i]; 
            boidObjects[i].position = boids[i].position;
            boidObjects[i].eulerAngles = boids[i].forward;
        }
    }


    private void Update()
    {
        // Render information for boidzero, useful for debugging forces and path planning
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
        
    }


    public void SetGoal(Vector3 goal)
    {
        if(!boidZeroNavigatingTowardGoal){
            boidZeroGoal = goal;
            boidZeroNavigatingTowardGoal = true;
        }
    }
}

