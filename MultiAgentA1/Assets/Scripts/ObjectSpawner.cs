using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;

public class ObjectSpawner : MonoBehaviour {
    // Use this for initialization
	public Material lineMaterial;
    public Material pathLineMaterial;
    public Material wallMaterial;
    public Problem problem;
    public string path;
    public float objectHeight = 4f;
    public float objectThickness = 0.5f;
    public GameObject goalObject;
    public GameObject vehicleObject;
	public Text timerText;
	public Text velocityText;
    public GameObject graphObject;
    public GameObject pathObject;
    public GameObject obstaclesObject;
    private float time;
	private float heightPoint;
    private float heightPointGoal;
    private Stack<Node> rrtPath;
    private Stack<Node> rrtPathCopy;
	private List<Node> rrtList;

    public bool drawTree = true;
    public bool drawPath = true;
    public bool removeGraphOnMove = false;
    public bool removePathOnMove = false;

    void Start () {
        problem = Problem.Import(path);
        spawnObjects();
        spawnActors();
		heightPoint = vehicleObject.transform.localScale.y / 2f;
        heightPointGoal = goalObject.transform.localScale.y / 2f;
        GameObject parent = this.transform.root.gameObject;
        RRT rrt = parent.GetComponent<RRT>();
        rrt.initialize(new Vector3(problem.pos_goal[0], heightPoint, problem.pos_goal[1]), new Vector3(problem.pos_start[0], heightPoint, problem.pos_start[1]), 
                            problem.vehicle_L, problem.vehicle_a_max, problem.vehicle_dt, problem.vehicle_omega_max, problem.vehicle_phi_max,
                            problem.vehicle_t, problem.vehicle_v_max, new Vector3(problem.vel_goal[0], 0, problem.vel_goal[1]), 
                            new Vector3(problem.vel_start[0], 0, problem.vel_start[1]), problem.obstacles, heightPoint);
		rrtPath = rrt.getPath ();
        if (drawTree)
        {
            rrtCompleted = false;
            rrtList = rrt.getTreeList();
        }
        if (drawPath)
        {
            rrtPathCopy = new Stack<Node>(rrtPath.Reverse());
            if(rrtPathCopy.Count != 0)
                rrtPathCopy.Pop();
            drawPathCompleted = false;
        }
        if (removeGraphOnMove)
            removeGraph = true;
        if (removePathOnMove)
            removePath = true;
        time = 0f;
    }


    Node target;
    Vector3 lastTarget;
    bool move = false;
	bool rrtCompleted = true;
    bool drawPathCompleted = true;
    int listIdx = 1;
    float currentTime = 0f;
    public float speedFactor = 10;
    bool render = false;
    bool removeGraph = false;
    bool removePath = false;

    // Update is called once per frame
    void Update () {
        if (render)
        {
            if (!rrtCompleted)
            {
                int nodesToDraw = Mathf.Max(1, (int)(listIdx * Time.deltaTime));
                int rrtListSize = rrtList.Count - 1;

                if (nodesToDraw + listIdx > rrtListSize)
                    nodesToDraw = rrtListSize - listIdx;

                if (listIdx <= rrtListSize - nodesToDraw)
                {
                    int i;
                    for (i = listIdx; i <= listIdx + nodesToDraw; i++)
                    {
                        GameObject tmp = new GameObject();
                        LineRenderer lineRenderer = tmp.AddComponent<LineRenderer>();
                        Node node = rrtList[i];
                        lineRenderer.material = lineMaterial;
                        lineRenderer.widthMultiplier = 0.1f;
                        lineRenderer.useWorldSpace = true;
                        lineRenderer.SetPosition(0, new Vector3(node.info.pos.x, 0, node.info.pos.z));
                        lineRenderer.SetPosition(1, new Vector3(node.parent.info.pos.x, 0, node.parent.info.pos.z));
                        //Instantiate(tmp);
                        tmp.transform.SetParent(graphObject.transform);
                    }
                    listIdx = i - 1;
                }

                if (listIdx == rrtListSize)
                    rrtCompleted = true;
                return;
            }

            if (!drawPathCompleted)
            {
                int nodesToDraw = Mathf.Max(1, (int)((rrtPath.Count - rrtPathCopy.Count) * Time.deltaTime)/2);
                if (nodesToDraw > rrtPathCopy.Count)
                    nodesToDraw = rrtPathCopy.Count;

                for(int i = 0; i < nodesToDraw; i++)
                {
                    GameObject tmp = new GameObject();
                    LineRenderer lineRenderer = tmp.AddComponent<LineRenderer>();
                    Node node = rrtPathCopy.Pop();
                    lineRenderer.material = pathLineMaterial;
                    lineRenderer.widthMultiplier = 0.2f;
                    lineRenderer.useWorldSpace = true;
                    lineRenderer.SetPosition(0, new Vector3(node.info.pos.x, 0.1f, node.info.pos.z));
                    lineRenderer.SetPosition(1, new Vector3(node.parent.info.pos.x, 0.1f, node.parent.info.pos.z));
                    //Instantiate(tmp);
                    tmp.transform.SetParent(pathObject.transform);
                }

                if (rrtPathCopy.Count == 0)
                    drawPathCompleted = true;
                return;
            }

            if (removeGraph)
            {
                removeChildren(graphObject);
                removeGraph = false;
                return;
            }

            if (removePath)
            {
                removeChildren(pathObject);
                removePath = false;
                return;
            }

            if (rrtPath.Count != 0 || move)
            {
                if (!move)
                {
                    target = rrtPath.Pop();
                    time += problem.vehicle_dt;
                    move = true;
                    vehicleObject.transform.LookAt(lastTarget + target.info.orientation);
                    currentTime = 0f;
                    //Update text
                    timerText.text = "Timer: " + time;
                    velocityText.text = "Velocity: [" + target.info.vel.x + "," + target.info.vel.z + "]";
                }
                currentTime += Time.deltaTime;
                vehicleObject.transform.Translate(speedFactor * (target.info.pos - lastTarget) * Time.deltaTime / problem.vehicle_dt, Space.World);
                if (currentTime >= problem.vehicle_dt / speedFactor)
                {
                    move = false;
                    lastTarget = target.info.pos;
                    vehicleObject.transform.position = lastTarget;
                }
            }
        }
    }

    void removeChildren(GameObject parent)
    {
        foreach (Transform child in parent.transform)
        {
            GameObject.Destroy(child.gameObject);
        }
    }

    public void startRender()
    {
        render = true;
    }

    void spawnActors()
    {
        goalObject.transform.position = new Vector3(problem.pos_goal[0], heightPointGoal, problem.pos_goal[1]);
        goalObject.transform.rotation = Quaternion.LookRotation(new Vector3(problem.vel_goal[0], 0, problem.vel_goal[1]));
        vehicleObject.transform.position = new Vector3(problem.pos_start[0], heightPoint, problem.pos_start[1]);
        vehicleObject.transform.rotation = Quaternion.LookRotation(new Vector3(problem.vel_start[0], 0, problem.vel_start[1]));
        lastTarget = vehicleObject.transform.position;
        Instantiate(goalObject);
    }

    void spawnObjects() {
        for(int i = 0; i < problem.obstacles.Count; i++)
        {
            if(problem.obstacles[i].type == PolygonType.bounding_polygon)
            {
                spawnObject(problem.obstacles[i], "bounding_polygon_"+i, obstaclesObject);
            }
            else
            {
                spawnObject(problem.obstacles[i], "obstacle_" + i, obstaclesObject);
            }
        }
    }

    void spawnObject(Polygon poly, string name, GameObject parent)
    {
        for(int i = 0; i < poly.corners.Count-1; i++)
        {
            Vector3 origin = new Vector3(poly.corners[i][0], objectHeight/2, poly.corners[i][1]);
            Vector3 destination = new Vector3(poly.corners[i+1][0], objectHeight/2, poly.corners[i + 1][1]);
            spawnWall(origin, destination, name + "_Side" + i, parent);
        }
        spawnWall(new Vector3(poly.corners[0][0], objectHeight/2, poly.corners[0][1]), new Vector3(poly.corners[poly.corners.Count-1][0], objectHeight/2, poly.corners[poly.corners.Count-1][1]), name+"_Side" + (poly.corners.Count - 1), parent);
    }

    void spawnWall(Vector3 origin, Vector3 destination, string name, GameObject parent)
    {
        float width = Vector3.Distance(origin, destination);
        Vector3 polygonSide = destination - origin;
        Vector3 middlePoint = origin + polygonSide * 0.5f;
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.transform.SetParent(parent.transform);
        wall.name = name;
        wall.transform.position = middlePoint;
		wall.GetComponent<Renderer> ().material = wallMaterial;
        wall.transform.rotation = Quaternion.LookRotation(polygonSide);
        wall.transform.localScale = new Vector3(objectThickness, objectHeight, width);
    }
}
