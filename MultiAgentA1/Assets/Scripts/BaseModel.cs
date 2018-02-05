using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class pointInfo
{
    public Vector3 pos;
    public Vector3 vel;
    public Vector3 orientation;
    public pointInfo(Vector3 pos, Vector3 vel, Vector3 orientation)
    {
        this.pos = pos;
        this.vel = vel;
        this.orientation = orientation;
    }
}

public abstract class BaseModel{

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
    public Vector3 orientation;
    public RRT rrt;

    // Use this for initialization
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    public abstract pointInfo moveTowards(pointInfo curPointInfo, Vector3 goalPos);

    public abstract Node getNearestVertex(List<Node> G, Vector3 qRand);

    public abstract List<Node> completePath(Node curPointNode, pointInfo goal);

    public BaseModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart, RRT rrt)
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
        this.orientation = velStart.normalized;
        this.rrt = rrt;
    }
}
