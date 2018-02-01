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
            //Debug.Log(contenders[i].info.orientation);
        }
        //Debug.Log("WInner: " +contenders[bestIndex].info.orientation);
        return contenders[bestIndex];
    }

    public DifferentialDriveModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart)
        : base(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart)
    {
    }
}