using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

    // Use this for initialization
    void Start ()
    {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    public void setParameters(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart)
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
    }

    public Stack<pointInfo> findPath()
    {
        KinematicModel kinematicModel = new KinematicModel(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart);
        Stack<pointInfo> path = new Stack<pointInfo>();
        pointInfo curPointInfo = new pointInfo(posStart, velStart);
        path.Push(curPointInfo);

        while (Vector3.Distance(curPointInfo.pos, posGoal) > 0.01)
        {
            curPointInfo = kinematicModel.moveTowards(curPointInfo, posGoal);
            path.Push(curPointInfo);
        }

        return new Stack<pointInfo>(path);
    }
}
