using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class pointInfo
{
    public Vector3 pos;
    public Vector3 vel;
    public pointInfo(Vector3 pos, Vector3 vel)
    {
        this.pos = pos;
        this.vel = vel;
    }
}

public abstract class BaseModel {

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
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}

    public abstract pointInfo moveTowards(pointInfo curPointInfo, Vector3 goalPos);

    public BaseModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart)
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
}
