using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class DifferentialDriveModel : BaseModel
{
    public int contendersCount = 10;

    public override pointInfo moveTowards(pointInfo curPointInfo, Vector3 targetPos)
    {
        Vector3 path = targetPos - curPointInfo.pos;
        float angle = Vector3.Angle(curPointInfo.orientation,path);
        float orientation = Vector3.Angle(Vector3.right, curPointInfo.orientation);
        if (curPointInfo.orientation.z < 0)
            orientation = -orientation;
        float newAngle = orientation * Mathf.Deg2Rad;
        if (angle > 0.00001)
        {
            Vector3 cross = Vector3.Cross(path, curPointInfo.orientation);
            float partTurn = omegaMax*Mathf.Rad2Deg * dt / angle;
            if (partTurn > 1f)
                partTurn = 1f;
            if (cross.y <= 0)
                angle = -angle;
            float turnAngle = angle * partTurn;
            newAngle = (turnAngle + orientation) * Mathf.Deg2Rad;
        }
        
        Vector3 newOrientation = new Vector3(Mathf.Cos(newAngle), 0, Mathf.Sin(newAngle));
        float xMove = vMax * Mathf.Cos(newAngle)*dt;
        float yMove = vMax * Mathf.Sin(newAngle)*dt;
        Vector3 newPosition = new Vector3(curPointInfo.pos.x + xMove, curPointInfo.pos.y, curPointInfo.pos.z + yMove);
        if(Vector3.Distance(newPosition,targetPos) > Vector3.Distance(curPointInfo.pos, targetPos))
        {
            newPosition = curPointInfo.pos;
            xMove = 0;
            yMove = 0;
        }
        
        float xVel = (float)Math.Round((Double)xMove / dt, 2, MidpointRounding.AwayFromZero);
        float zVel = (float)Math.Round((Double)yMove / dt, 2, MidpointRounding.AwayFromZero);
        return new pointInfo(newPosition, new Vector3(xVel, 0, zVel), newOrientation);
    }

    public override Node getNearestVertex(List<Node> G, Vector3 qRand)
    {
        Node[] contenders = new Node[Mathf.Min(contendersCount, G.Count)];
        float[] contendersDist = new float[Mathf.Min(contendersCount, G.Count)];
        int replaceIndex = 0;
        float maxDist = 0;
        for (int i = 0; i < contenders.Length; i++)
        {
            contenders[i] = G[i];
            contendersDist[i] = Vector3.Distance(G[i].info.pos, qRand);
            if (contendersDist[i] > maxDist)
            {
                replaceIndex = i;
                maxDist = contendersDist[i];
            }
        }
        
        for (int i = contenders.Length; i < G.Count; i++)
        {
            float curDist = Vector3.Distance(G[i].info.pos, qRand);
            if (curDist < maxDist)
            {
                contenders[replaceIndex] = G[i];
                contendersDist[replaceIndex] = curDist;
                maxDist = 0;
                for(int j = 0; j < contenders.Length; j++)
                {
                    if(contendersDist[j] > maxDist)
                    {
                        replaceIndex = j;
                        maxDist = contendersDist[j];
                    }
                }
            }
        }

        float minAngle = Vector3.Angle(qRand - contenders[0].info.pos, contenders[0].info.orientation);
        int bestIndex = 0;
        for(int i = 0; i < contenders.Length; i++)
        {
            float angle = Vector3.Angle(qRand - contenders[i].info.pos, contenders[i].info.orientation);
            if (angle < minAngle)
            {
                minAngle = angle;
                bestIndex = i;
            }
        }
        return contenders[bestIndex];
    }

    public override List<Node> completePath(Node curPointNode, pointInfo goal)
    {
        List<Node> nodeList = new List<Node>();

        //if distance from goal is not within one timestep
        if(Vector3.Distance(curPointNode.info.pos, goal.pos) > vMax * dt)
            return null;
      
        float angle = Vector3.Angle(curPointNode.info.orientation, goal.orientation);
        float orientation = Vector3.Angle(Vector3.right, curPointNode.info.orientation); 
        if (curPointNode.info.orientation.z < 0)
            orientation = -orientation;
        float turningDts = angle / (dt * omegaMax * Mathf.Rad2Deg); 
        int turningDtCount = (int)Mathf.Ceil(turningDts); // count of dts needed to turn to the goal's orientation

        Vector3 cross = Vector3.Cross(goal.orientation, curPointNode.info.orientation);
        if (cross.y <= 0f)
            angle = -angle;


        Node parent = curPointNode;
        for (int i = 1; i < turningDtCount; i++)
        {
            float newAngle = (orientation + i * omegaMax * Mathf.Rad2Deg * dt * Mathf.Sign(angle)) * Mathf.Deg2Rad;
            Vector3 newOrientation = new Vector3(Mathf.Cos(newAngle), 0, Mathf.Sin(newAngle));
            nodeList.Add(new Node(parent, new pointInfo(parent.info.pos, new Vector3(0f, 0f, 0f), newOrientation)));
            parent = nodeList[nodeList.Count - 1];
        }

        nodeList.Add(new Node(parent, new pointInfo(goal.pos, goal.vel, goal.orientation)));
        return nodeList;
    }

    public DifferentialDriveModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart, RRT rrt)
        : base(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart, rrt)
    {
    }
}