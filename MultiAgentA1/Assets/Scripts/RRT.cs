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
	public float vehicleL;
	public float vehicleW;
    public BaseModel motionModel;
    private float xmin, xmax;
    private float ymin, ymax;
    private Stack<Node> path;
	private List<Node> G;
    public int nrOfnodes = 50000;
	public bool bCollision = false;

    // Use this for initialization
    void Start ()
    {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}


	public Stack<Node> getPath(){
        if (path == null)
            path = new Stack<Node>();
        return path;
    }
	public List<Node> getTreeList(){
        if (G == null)
            G = new List<Node>();
        return G;
    }

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

    private void buildRRT(int minNodes, int maxNodes)
    {
        Node root = new Node(null, new pointInfo(motionModel.posStart, motionModel.velGoal, motionModel.orientation));
        G = new List<Node>();
        G.Add(root);
        int seed = 0;
        for(int k = 0; k < maxNodes; k++)
        {
            Vector3 qRand = posGoal;
            if (seed++ % 20 != 0)
                qRand = new Vector3(Random.Range(xmin, xmax), heightPoint, Random.Range(ymin, ymax));
			Node qNear = motionModel.getNearestVertex(G, qRand);
            Node qNew = new Node(qNear, motionModel.moveTowards(qNear.info, qRand));

			//Check for collisions
			if (!bCollision) 
			{
				if (insideObstacle(qNew.info.pos))
				{
					k--;
					continue;
				}
			}
			else 
			{
				// if bColission is true, the car's size is taken into account
				if (insideObstacle(qNew.info.pos + new Vector3(0, 0, this.vehicleL/2)) 
					|| insideObstacle(qNew.info.pos + new Vector3(0, 0, -this.vehicleL/2))
					|| insideObstacle(qNew.info.pos+ new Vector3(this.vehicleW/2, 0, 0))  
					|| insideObstacle(qNew.info.pos + new Vector3(-this.vehicleW/2, 0, 0)))
				{
					k--;
					continue;
				}
			}


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

	public void initialize(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart, List<Polygon> obstacles, float heightPoint, float vehicleL, float vehicleW)
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
		this.vehicleL = vehicleL;
		this.vehicleW = vehicleW;
		this.motionModel = new KinematicModel(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart);
        calculateSpace();
		buildRRT (0, nrOfnodes);
    }

    private bool insideObstacle(Vector3 point)
    {
        for(int i = 0; i < obstacles.Count; i++)
        {
            Polygon poly = obstacles[i];
            if (poly.type == PolygonType.obstacle && insidePolygon(poly, point))
                return true;
            else if (poly.type == PolygonType.bounding_polygon && !insidePolygon(poly, point))
                return true;
        }
        return false;
    }

	private bool insidePolygon(Polygon poly, Vector3 point)
	{
		List<float[]> vertices = poly.corners;
		int vertexCount = vertices.Count;
        float[] p = { point.x, point.z };
        
        
        if (vertexCount < 3)
			return false; // not a polygon

        return isOdd(vertices, p, vertexCount);

    }

	private bool isOdd(List<float[]> vertices, float[] point, int vertexCount)
	{
		float[] extremePoint = { xmax * 2f, point[1] }; // point needed to draw line towards infinity
		int intersectCount = 0; //number of times our line towards infinity intersects a polygon side
		int i = 0;

        do {

			int next = (i + 1) % vertexCount; //get Index of the next vertex of the polygon

			//if the line to infinity intersects this side
			if(intersectsSide(vertices[i], vertices[next], point, extremePoint)) {
				//if this side has slope 0
				if(getOrientation(vertices[i], point, vertices[next]) == 0) 
					return onSegment(vertices[i], point, vertices[next]);

				intersectCount++;
			}

			i = next;		
		} while(i != 0);

        return intersectCount % 2 == 1;
	}


	private bool intersectsSide(float[] vertex1, float[] vertex2, float[] point, float[] extremePoint)
	{
		int o1, o2, o3, o4;
 
		o1 = getOrientation (vertex1, vertex2, point); 
		o2 = getOrientation (vertex1, vertex2, extremePoint); 
		o3 = getOrientation (point, extremePoint, vertex1); 
		o4 = getOrientation (point, extremePoint, vertex2);

        if (o1 != o2 && o3 != o4)
            return true;

        if (o1 == 0 && onSegment(vertex1, point, vertex2))
            return true;
        if (o2 == 0 && onSegment(vertex1, extremePoint, vertex2))
            return true;
        if (o3 == 0 && onSegment(point, vertex1, extremePoint))
            return true;
        if (o4 == 0 && onSegment(point, vertex2, extremePoint))
            return true;

        return false;
    }

    //change floats to vector3
    private int getOrientation(float[] vertex1, float[] vertex2, float[] vertex3)
	{
		float orientation = (vertex2 [1] - vertex1 [1]) * (vertex3 [0] - vertex2 [0]) -
		                  (vertex2 [0] - vertex1 [0]) * (vertex3 [1] - vertex2 [1]);
		if (orientation == 0f)
			return 0;
		else if (orientation > 0f)
			return 1;
		else
			return 2;
	}

    private bool onSegment(float[] vertex1, float[] vertex2, float[] vertex3)
    {
        if (vertex2[0] <= Mathf.Max(vertex1[0], vertex3[0]) && vertex2[0] >= Mathf.Min(vertex1[0], vertex3[0])
            && vertex2[1] <= Mathf.Max(vertex1[1], vertex3[1]) && vertex2[1] >= Mathf.Min(vertex1[1], vertex3[1]))
            return true;
        return false;
    }
}
