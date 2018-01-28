using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectSpawner : MonoBehaviour {
    // Use this for initialization
    public Problem problem;
    public string path;
    public float objectHeight = 4f;
    public float objectThickness = 0.5f;
    public GameObject goalObject;
    public GameObject vehicleObject;
    float heightPoint;
    Stack<pointInfo> rrtPath;

    void Start () {
        problem = Problem.Import(path);
        spawnObjects();
        spawnActors();
        heightPoint = vehicleObject.transform.localScale.y / 2;
        GameObject parent = this.transform.root.gameObject;
        RRT rrt = parent.GetComponent<RRT>();
        rrt.initialize(new Vector3(problem.pos_goal[0], heightPoint, problem.pos_goal[1]), new Vector3(problem.pos_start[0], heightPoint, problem.pos_start[1]), 
                            problem.vehicle_L, problem.vehicle_a_max, problem.vehicle_dt, problem.vehicle_omega_max, problem.vehicle_phi_max,
                            problem.vehicle_t, problem.vehicle_v_max, new Vector3(problem.vel_goal[0], 0, problem.vel_goal[1]), 
                            new Vector3(problem.vel_start[0], 0, problem.vel_start[1]), problem.obstacles, heightPoint);
        rrtPath = rrt.findPath();
        Debug.Log(rrtPath.Count);
    }

    Vector3 target;
    Vector3 lastTarget;
    bool move = false;
	// Update is called once per frame
	void Update () {
        if (rrtPath.Count != 0 || move)
        {
            if (!move)
            {
                lastTarget = vehicleObject.transform.position;
                target = rrtPath.Pop().pos;
                move = true;
            }
            vehicleObject.transform.Translate((target- lastTarget) * Time.deltaTime/problem.vehicle_dt, Space.World);
            if (Vector3.Distance(target,vehicleObject.transform.position) <= 0.5)
            {
                move = false;
            }
        }
    }

    void spawnActors()
    {
        goalObject.transform.position = new Vector3(problem.pos_goal[0], 1, problem.pos_goal[1]);
        goalObject.transform.rotation = Quaternion.LookRotation(new Vector3(problem.vel_goal[0], 0, problem.vel_goal[1]));
        vehicleObject.transform.position = new Vector3(problem.pos_start[0], heightPoint, problem.pos_start[1]);
        vehicleObject.transform.rotation = Quaternion.LookRotation(new Vector3(problem.vel_start[0], 0, problem.vel_start[1]));
        Instantiate(goalObject);
        //(vehicleObject);
    }

    void spawnObjects() {
        for(int i = 0; i < problem.obstacles.Count; i++)
        {
            if(problem.obstacles[i].type == PolygonType.bounding_polygon)
            {
                spawnObject(problem.obstacles[i], "bounding_polygon_"+i);
            }
            else
            {
                spawnObject(problem.obstacles[i], "obstacle_" + i);
            }
        }
    }

    void spawnObject(Polygon poly, string name)
    {
        for(int i = 0; i < poly.corners.Count-1; i++)
        {
            Vector3 origin = new Vector3(poly.corners[i][0], objectHeight/2, poly.corners[i][1]);
            Vector3 destination = new Vector3(poly.corners[i+1][0], objectHeight/2, poly.corners[i + 1][1]);
            spawnWall(origin, destination, name + "_Side" + i);
        }
        spawnWall(new Vector3(poly.corners[0][0], objectHeight/2, poly.corners[0][1]), new Vector3(poly.corners[poly.corners.Count-1][0], objectHeight/2, poly.corners[poly.corners.Count-1][1]), name+"_Side" + (poly.corners.Count - 1));
    }

    void spawnWall(Vector3 origin, Vector3 destination, string name)
    {
        float width = Vector3.Distance(origin, destination);
        Vector3 polygonSide = destination - origin;
        Vector3 middlePoint = origin + polygonSide * 0.5f;
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = name;
        wall.transform.position = middlePoint;
        wall.transform.rotation = Quaternion.LookRotation(polygonSide);
        wall.transform.localScale = new Vector3(objectThickness, objectHeight, width);
    }
}
