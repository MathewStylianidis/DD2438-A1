using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
	public Node parent;
	public pointInfo info;
	private List<Node> children;

	public Node(Node parent, pointInfo info)
	{
		this.parent = parent;
		this.info = info;
		this.children = new List<Node>();
	}

	public Node getChild(int idx) { return children[idx];}
	public void appendChild(Node child) { children.Add(child);}

}


public class RRT : MonoBehaviour {

    public Vector3 posGoal;
    public Vector3 posStart;
    public float length;
    public float aMax;
    public float dt;
    public float omegaMax;
    public float phiMax;
    public float t;
    public float vMax;
    public Vector3 velGoal;
    public Vector3 velStart;
    public List<Polygon> obstacles;
    public float heightPoint;
    public BaseModel motionModel;
    private float xmin, xmax;
    private float ymin, ymax;
    private Stack<pointInfo> path;
	private List<Node> G;
  

    // Use this for initialization
    void Start ()
    {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}


	public Stack<pointInfo> getPath(){ return path;}
	public List<Node> getTreeList(){ return G;}

    private void calculateSpace()
    {
        for (int i = 0; i < obstacles.Count; i++)
        {
            if (obstacles[i].type == PolygonType.bounding_polygon)
            {
                ymin = xmin = float.MaxValue;
                xmax = ymax = float.MinValue;

                foreach (float[] vertex in obstacles[i].corners)
                {
                    if (vertex[0] < xmin)
                        xmin = vertex[0];
                    else if (vertex[0] > xmax)
                        xmax = vertex[0];

                    if (vertex[1] < ymin)
                        ymin = vertex[1];
                    else if (vertex[1] > ymax)
                        ymax = vertex[1];
                }
                return;
            }
        }
    }

    private Node getNearestVertex(List<Node> G, Vector3 qRand)
    {
        Node minNode = G[0];
        float minDistance = Vector3.Distance(minNode.info.pos, qRand);
        float curDist;
        for (int i = 1; i < G.Count; i++)
        {
            curDist = Vector3.Distance(G[i].info.pos, qRand);
            if (curDist < minDistance)
            {
                minDistance = curDist;
                minNode = G[i];
            }
        }
        return minNode;
    }

    private void buildRRT(int minNodes, int maxNodes)
    {
        Node root = new Node(null, new pointInfo(posStart, velStart));
        G = new List<Node>();
        G.Add(root);
        
        for(int k = 0; k < maxNodes; k++)
        {
			Vector3 qRand = new Vector3(Random.Range(xmin, xmax), heightPoint, Random.Range(ymin, ymax));
			Node qNear;
			if (k % 10 != 0)
				qNear = getNearestVertex (G, qRand);
			else
				qNear = getNearestVertex (G, posGoal);
            Node qNew = new Node(qNear, motionModel.moveTowards(qNear.info, qRand));
            G.Add(qNew);
            if(Vector3.Distance(qNew.info.pos, posGoal) < vMax * dt)
            {
                Stack<pointInfo> path = new Stack<pointInfo>();
                Node curNode = qNew;
  
                while (curNode != null)
                {
                    path.Push(curNode.info);
                    curNode = curNode.parent;
                }
                this.path = path;
				return;
            }
        }

    }

    public void initialize(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart, List<Polygon> obstacles, float heightPoint)
    {
        this.posGoal = posGoal;
        this.posStart = posStart;
        this.length = length;
        this.aMax = aMax;
        this.dt = dt;
        this.omegaMax = omegaMax;
        this.phiMax = phiMax;
        this.t = t;
        this.vMax = vMax;
        this.velGoal = velGoal;
        this.velStart = velStart;
        this.obstacles = obstacles;
        this.heightPoint = heightPoint;
        this.motionModel = new KinematicModel(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart);
        calculateSpace();
		buildRRT (0, int.MaxValue);
    }




    public Stack<pointInfo> findPath()
    {
        KinematicModel kinematicModel = new KinematicModel(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart);
        Stack<pointInfo> path = new Stack<pointInfo>();
        pointInfo curPointInfo = new pointInfo(posStart, velStart);
        //path.Push(curPointInfo);

        while (Vector3.Distance(curPointInfo.pos, posGoal) > 0.01)
        {
            curPointInfo = kinematicModel.moveTowards(curPointInfo, posGoal);
            path.Push(curPointInfo);
        }

        return new Stack<pointInfo>(path);
    }
}
