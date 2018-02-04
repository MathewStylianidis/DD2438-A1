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
		return new pointInfo(curPointInfo.pos + newPath, new Vector3(xVel , 0, zVel), Vector3.Normalize(path));
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

    public override List<Node> completePath(Node curPointNode, pointInfo goal)
    {
        Vector3 path = goal.pos - curPointNode.info.pos;
        float amountOfDts = Vector3.Distance(goal.pos, curPointNode.info.pos) / (dt * vMax);
        int dtCount = (int)Mathf.Ceil(amountOfDts);
        Vector3 dir = path.normalized;
        List<Node> nodeList = new List<Node>();
        Vector3 distanceDt = dir * vMax * dt;
        Node parent = curPointNode;
        for (int i = 1; i < dtCount; i++)
        {
            Vector3 newPos = curPointNode.info.pos + distanceDt * i;
            if (rrt.insideObstacle(newPos))
                return null;
            nodeList.Add(new Node(parent, new pointInfo(newPos, dir * vMax, dir)));
            parent = nodeList[i - 1];
        }
        nodeList.Add(new Node(parent, new pointInfo(goal.pos, goal.vel, goal.orientation)));
        return nodeList;
    }

    public KinematicModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart, RRT rrt) 
        :base(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart, rrt) 
    {     
    }
}
