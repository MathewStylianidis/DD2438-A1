using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ObjectSpawner : MonoBehaviour {
    // Use this for initialization
	public Material lineMaterial;
	public Material wallMaterial;
    public Problem problem;
    public string path;
    public float objectHeight = 4f;
    public float objectThickness = 0.5f;
    public GameObject goalObject;
    public GameObject vehicleObject;
	public Text timerText;
	public Text velocityText;
	private float time;
	private float heightPoint;
	private Stack<Node> rrtPath;
	private List<Node> rrtList;




    void Start () {
        problem = Problem.Import(path);
        spawnObjects();
        spawnActors();
		heightPoint = vehicleObject.transform.localScale.y / 2f;
        GameObject parent = this.transform.root.gameObject;
        RRT rrt = parent.GetComponent<RRT>();
        rrt.initialize(new Vector3(problem.pos_goal[0], heightPoint, problem.pos_goal[1]), new Vector3(problem.pos_start[0], heightPoint, problem.pos_start[1]), 
                            problem.vehicle_L, problem.vehicle_a_max, problem.vehicle_dt, problem.vehicle_omega_max, problem.vehicle_phi_max,
                            problem.vehicle_t, problem.vehicle_v_max, new Vector3(problem.vel_goal[0], 0, problem.vel_goal[1]), 
                            new Vector3(problem.vel_start[0], 0, problem.vel_start[1]), problem.obstacles, heightPoint);
		rrtPath = rrt.getPath ();
		rrtList = rrt.getTreeList ();
		time = 0f;
    }


    Node target;
    Vector3 lastTarget;
    bool move = false;
	bool rrtCompleted = false;
	int listIdx = 1;


	// Update is called once per frame
	void Update () {	
		if (!rrtCompleted) 
		{
            int nodesToDraw = Mathf.Max(1,(int)(listIdx * Time.deltaTime));
			if (listIdx < rrtList.Count- nodesToDraw) 
			{
                for (int i = 0; i < nodesToDraw; i++)
                {
                    GameObject tmp = new GameObject();
                    LineRenderer lineRenderer = tmp.AddComponent<LineRenderer>();
                    Node node = rrtList[listIdx];
                    lineRenderer.material = lineMaterial;
                    lineRenderer.widthMultiplier = 0.1f;
                    lineRenderer.useWorldSpace = true;
                    lineRenderer.SetPosition(0, new Vector3(node.info.pos.x, 0, node.info.pos.z));
                    lineRenderer.SetPosition(1, new Vector3(node.parent.info.pos.x, 0, node.parent.info.pos.z));
                    Instantiate(tmp);
                    listIdx++;
                }
				return;
			} 
			else
			{
				rrtCompleted = true;
			}
		}

        if (rrtPath.Count != 0 || move)
        {
            if (!move)
            {
                lastTarget = vehicleObject.transform.position;
                target = rrtPath.Pop();
				time += problem.vehicle_dt;
                move = true;
                vehicleObject.transform.LookAt(target.info.pos);
                //Update text
                timerText.text = "Timer: " + time;
				velocityText.text = "Velocity: [" + target.info.vel.x + "," + target.info.vel.z + "]";
            }
			vehicleObject.transform.Translate((target.info.pos - lastTarget) * Time.deltaTime/problem.vehicle_dt, Space.World);
			if (Vector3.Distance(target.info.pos, vehicleObject.transform.position) < problem.vehicle_dt*problem.vehicle_v_max)
                move = false;

        }
    }

    void spawnActors()
    {
        goalObject.transform.position = new Vector3(problem.pos_goal[0], 1, problem.pos_goal[1]);
        goalObject.transform.rotation = Quaternion.LookRotation(new Vector3(problem.vel_goal[0], 0, problem.vel_goal[1]));
        vehicleObject.transform.position = new Vector3(problem.pos_start[0], heightPoint, problem.pos_start[1]);
        vehicleObject.transform.rotation = Quaternion.LookRotation(new Vector3(problem.vel_start[0], 0, problem.vel_start[1]));
        Instantiate(goalObject);
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
		wall.GetComponent<Renderer> ().material = wallMaterial;
        wall.transform.rotation = Quaternion.LookRotation(polygonSide);
        wall.transform.localScale = new Vector3(objectThickness, objectHeight, width);
    }
}
