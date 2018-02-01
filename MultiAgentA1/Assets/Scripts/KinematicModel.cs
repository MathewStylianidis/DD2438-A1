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
            partDist = 1f;
        Vector3 newPath = path * partDist;
        float xVel = (float)Math.Round((Double)newPath.x/dt, 2, MidpointRounding.AwayFromZero);
        float zVel = (float)Math.Round((Double)newPath.z/dt, 2, MidpointRounding.AwayFromZero);
        return new pointInfo(curPointInfo.pos + newPath, new Vector3(xVel , 0, zVel), path);
    }

    public override Node getNearestVertex(List<Node> G, Vector3 qRand)
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

    public KinematicModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart) 
        :base(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart) 
    {     
    }
}
