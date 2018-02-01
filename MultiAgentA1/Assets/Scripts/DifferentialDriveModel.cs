using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class DifferentialDriveModel : BaseModel
{
    public int contendersCount = 5;

    public override pointInfo moveTowards(pointInfo curPointInfo, Vector3 targetPos)
    {
        float angle = Vector3.Angle(curPointInfo.orientation, targetPos - curPointInfo.pos);
        Vector3 cross = Vector3.Cross(targetPos - curPointInfo.pos, curPointInfo.orientation);
        float partTurn = omegaMax*dt/angle;
        if(1 > partTurn)
            partTurn = 1;
        if (cross.y < 0)
            angle = -angle;
        float turnAngle = angle * partTurn*Mathf.Deg2Rad;
        float orientation = Vector3.Angle(Vector3.right, curPointInfo.orientation);
        Vector3 newOrientation = new Vector3(Mathf.Cos(turnAngle + orientation), 0, Mathf.Sin(turnAngle + orientation));
        float xMove = vMax * Mathf.Cos(turnAngle + orientation);
        float yMove = vMax * Mathf.Sin(turnAngle + orientation);
        Vector3 newPosition = new Vector3(curPointInfo.pos.x + xMove, 0, curPointInfo.pos.y + yMove);
        if(Vector3.Distance(newPosition,targetPos) > Vector3.Distance(curPointInfo.pos, targetPos))
        {
            newPosition = new Vector3(curPointInfo.pos.x, 0, curPointInfo.pos.y);
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
                float newMaxDist = 0;
                for(int j = 0; j < contenders.Length; j++)
                {
                    if(contendersDist[j] > newMaxDist)
                    {
                        replaceIndex = j;
                        maxDist = newMaxDist;
                    }
                }
            }
        }

        float minAngle = Vector3.Angle(qRand - contenders[0].info.pos, contenders[0].info.orientation);
        int bestIndex = 0;
        for(int i = 0; i < contenders.Length; i++)
        {
            float angle = Vector3.Angle(qRand - contenders[i].info.pos, contenders[i].info.orientation);
            if(angle < minAngle)
            {
                minAngle = angle;
                bestIndex = i;
            }
        }

        return contenders[bestIndex];
    }

    public DifferentialDriveModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart)
        : base(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart)
    {
    }
}