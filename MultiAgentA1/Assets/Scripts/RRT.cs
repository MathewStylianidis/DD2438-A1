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
    private Stack<Node> path;
	private List<Node> G;
  

    // Use this for initialization
    void Start ()
    {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}


	public Stack<Node> getPath(){ return path;}
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
                Stack<Node> path = new Stack<Node>();
                Node curNode = qNew;
  
                while (curNode != null)
                {
                    path.Push(curNode);
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


	private bool insidePolygon(Polygon poly, Vector3 point)
	{
		List<float[]> vertices = poly.corners;
		int vertexCount = vertices.Count;
		int intersectCount = 0; //number of times our line towards infinity intersects a polygon side

		if (vertexCount < 3)
			return false; // not a polygon
		
		//intersectCount = getIntersectCount (vertices, point, vertexCount);
		return intersectCount % 2 == 1;
	}

	/*private int getIntersectCount(List<float[]> vertices, Vector3 point, int vertexCount)
	{
		Vector3 extremePoint = new Vector3 (float.MaxValue, 0, point.z); // point needed to draw line towards infinity
		int intersectCount = 0; //number of times our line towards infinity intersects a polygon side
		int i = 0;

		do {

			int next = (i + 1) / vertexCount; //get Index of the next vertex of the polygon

			//if the line to infinity intersects this side
			if(intersectsSide(vertices[i], vertices[next], point, extremePoint)) {
				//if this side has slope 0
				//if(getOrientation()) 
					//return onSegment();

				vertexCount++;
			}

			i = next;		
		} while(i != 0);

	}*/


	/*private bool intersectsSide(float[] vertex1, float[] vertex2, Vector3 point, Vector3 extremePoint)
	{
		int o1, o2, o3, o4;


		//o1 = getOrientation (vertex1, vertex2, point); 
		//o1 = getOrientation (vertex1, vertex2, extremePoint); 
		//o1 = getOrientation (point, extremePoint, vertex1); 
		//o1 = getOrientation (point, extremePoint, point);  
	}*/

	//change floats to vector3
	/*private int getOrientation(float[] vertex1, float[] vertex2, float[] vertex3)
	{
		int orientation = (vertex2 [1] - vertex1 [1]) * (vertex3 [0] - vertex2 [0]) -
		                  (vertex2 [0] - vertex1 [0]) * (vertex3 [1] - vertex2 [1]);
		if (orientation == 0)
			return 0;
		else if (orientation > 0)
			return 1;
		else
			return 2;
	}*/
}
