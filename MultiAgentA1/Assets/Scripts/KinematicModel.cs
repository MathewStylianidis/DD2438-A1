using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinematicModel : BaseModel
{
    public override pointInfo moveTowards(pointInfo curPointInfo, Vector3 targetPos)
    {
        Vector3 path = targetPos - curPointInfo.pos;
        float dist = Vector3.Distance(targetPos, curPointInfo.pos);
        float m = vMax* dt;
        float partDist = m / dist;
        if (partDist >= 1.0)
        {
            partDist = 1f;
        }
        Vector3 newPath = path * partDist;
        return new pointInfo(curPointInfo.pos + newPath, new Vector3((newPath.x- curPointInfo.pos.x)/dt, 0, (newPath.z - curPointInfo.pos.z) / dt));
    }

    public KinematicModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart) 
        :base(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart) 
    {     
    }
}
