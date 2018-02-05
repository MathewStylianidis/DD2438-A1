using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class DynamicPointModel : BaseModel
{
    public int contendersCount = 10;

    public override pointInfo moveTowards(pointInfo curPointInfo, Vector3 targetPos)
    {
        Vector3 path = targetPos - curPointInfo.pos;
        Vector3 deltaVel = path.normalized*aMax*dt;
        Vector3 newVel = Vector3.ClampMagnitude(curPointInfo.vel+deltaVel, vMax);
        float xMove = newVel.x * dt;
        float yMove = newVel.z * dt;
        Vector3 newPosition = new Vector3(curPointInfo.pos.x + xMove, curPointInfo.pos.y, curPointInfo.pos.z + yMove);
        Vector3 newOrientation = path.normalized;
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
                for (int j = 0; j < contenders.Length; j++)
                {
                    if (contendersDist[j] > maxDist)
                    {
                        replaceIndex = j;
                        maxDist = contendersDist[j];
                    }
                }
            }
        }

        float minAngle = Vector3.Angle(qRand - contenders[0].info.pos, contenders[0].info.orientation);
        int bestIndex = 0;
        for (int i = 0; i < contenders.Length; i++)
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
        float radius1 = Mathf.Pow(curPointNode.info.vel.magnitude,2) / aMax;
        float radius2 = Mathf.Pow(goal.vel.magnitude, 2) / aMax;
        List<List<Node>> pathLists = new List<List<Node>>();
        pathLists.Add(RL(curPointNode, goal, radius1, radius2));
        pathLists.Add(RR(curPointNode, goal, radius1, radius2));
        pathLists.Add(LR(curPointNode, goal, radius1, radius2));
        pathLists.Add(LL(curPointNode, goal, radius1, radius2));
        int minIdx = 0;
        int minSize = int.MaxValue;
        for (int i = 0; i < pathLists.Count; i++)
            if (pathLists[i] != null && pathLists[i].Count < minSize)
            {
                minIdx = i;
                minSize = pathLists[i].Count;
            }
        return pathLists[minIdx];
    }

    private List<Node> RL(Node curPointNode, pointInfo goal, float radius1, float radius2)
    {
        Vector3 dir = Vector3.Cross(Vector3.up, curPointNode.info.vel).normalized;
        Vector3 p1 = curPointNode.info.pos + dir * radius1;
        Vector3 dir2 = Vector3.Cross(Vector3.up, goal.vel).normalized;
        Vector3 p2 = goal.pos - dir2 * radius2;

        float D = (p2 - p1).magnitude;
        float alpha = radius1 + radius2;
        float theta = Mathf.Acos(alpha / D);
        float baseAngle = Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
        if ((p2 - p1).z < 0)
            baseAngle = -baseAngle;
        Vector3 tp1 = p1 + new Vector3(Mathf.Cos(theta + baseAngle), 0, Mathf.Sin(theta + baseAngle)) * radius1;
        Vector3 tp2 = p2 - new Vector3(Mathf.Cos(theta + baseAngle), 0, Mathf.Sin(theta + baseAngle)) * radius2;

        return dubinPath(curPointNode, goal, radius1, radius2, tp1, tp2, p1, p2, true, false);
    }

    private List<Node> LR(Node curPointNode, pointInfo goal, float radius1, float radius2)
    {
        Vector3 dir = Vector3.Cross(Vector3.up, curPointNode.info.vel).normalized;
        Vector3 p1 = curPointNode.info.pos - dir * radius1;
        Vector3 dir2 = Vector3.Cross(Vector3.up, goal.vel).normalized;
        Vector3 p2 = goal.pos + dir2 * radius2;

        float D = (p2 - p1).magnitude;
        float alpha = radius1 + radius2;
        float theta = -Mathf.Acos(alpha / D);
        float baseAngle = Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
        if ((p2 - p1).z < 0)
            baseAngle = -baseAngle;
        Vector3 tp1 = p1 + new Vector3(Mathf.Cos(theta + baseAngle), 0, Mathf.Sin(theta + baseAngle)) * radius1;
        Vector3 tp2 = p2 - new Vector3(Mathf.Cos(theta + baseAngle), 0, Mathf.Sin(theta + baseAngle)) * radius2;

        return dubinPath(curPointNode, goal, radius1, radius2, tp1, tp2, p1, p2, false, true);
    }

    private List<Node> RR(Node curPointNode, pointInfo goal, float radius1, float radius2)
    {
        Vector3 dir = Vector3.Cross(Vector3.up, curPointNode.info.vel).normalized;
        Vector3 p1 = curPointNode.info.pos + dir * radius1;
        Vector3 dir2 = Vector3.Cross(Vector3.up, goal.vel).normalized;
        Vector3 p2 = goal.pos + dir2 * radius2;

        float D = (p2 - p1).magnitude;
        float H = Mathf.Sqrt(Mathf.Pow(D,2) - Mathf.Pow(radius1-radius2,2));
        float Y = Mathf.Sqrt(Mathf.Pow(H,2) + Mathf.Pow(radius2, 2));
        float theta = Mathf.Acos((Mathf.Pow(radius1, 2) + Mathf.Pow(D, 2) - Mathf.Pow(Y, 2)) / (2 * radius1 * D));
        float baseAngle = Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
        if ((p2-p1).z < 0)
            baseAngle = -baseAngle;
        Vector3 tp1 = p1 + new Vector3(Mathf.Cos(theta+ baseAngle), 0, Mathf.Sin(theta+ baseAngle)) * radius1;
        Vector3 tp2 = p2 + new Vector3(Mathf.Cos(theta+ baseAngle), 0, Mathf.Sin(theta+ baseAngle)) * radius2;
        
        return dubinPath(curPointNode, goal, radius1, radius2, tp1, tp2, p1, p2, true, true);
    }

    private List<Node> LL(Node curPointNode, pointInfo goal, float radius1, float radius2)
    {
        Vector3 dir = Vector3.Cross(Vector3.up, curPointNode.info.vel).normalized;
        Vector3 p1 = curPointNode.info.pos - dir * radius1;
        Vector3 dir2 = Vector3.Cross(Vector3.up, goal.vel).normalized;
        Vector3 p2 = goal.pos - dir2 * radius2;

        float D = (p2 - p1).magnitude;
        float H = Mathf.Sqrt(Mathf.Pow(D, 2) - Mathf.Pow(radius1 - radius2, 2));
        float Y = Mathf.Sqrt(Mathf.Pow(H, 2) + Mathf.Pow(radius2, 2));
        float theta = Mathf.Acos((Mathf.Pow(radius1, 2) + Mathf.Pow(D, 2) - Mathf.Pow(Y, 2)) / (2 * radius1 * D));
        float baseAngle = Vector3.Angle(Vector3.right, p2 - p1) * Mathf.Deg2Rad;
        if ((p2 - p1).z < 0)
            baseAngle = -baseAngle;
        Vector3 tp1 = p1 + new Vector3(Mathf.Cos(-theta + baseAngle), 0, Mathf.Sin(-theta + baseAngle)) * radius1;
        Vector3 tp2 = p2 + new Vector3(Mathf.Cos(-theta + baseAngle), 0, Mathf.Sin(-theta + baseAngle)) * radius2;

        return dubinPath(curPointNode, goal, radius1, radius2, tp1, tp2, p1, p2, false, false);
    }


    private List<Node> dubinPath(Node curPointNode, pointInfo goal, float radius1, float radius2, Vector3 tp1, Vector3 tp2, Vector3 p1, Vector3 p2, bool rightStart, bool rightGoal)
    {
        Vector3 tangentLine = tp2 - tp1;

        if (rrt.insideObstacle(tp1) || rrt.insideObstacle(tp2))
            return null;
        float arcLength = getArcLength(curPointNode.info.pos - p1, tp1 - p1, rightStart, radius1);

        float movingDts = arcLength / (dt * curPointNode.info.vel.magnitude);
        int moveCount = (int)Mathf.Ceil(movingDts);
        float phi1 = arcLength / radius1;
        float angleStep = phi1 / moveCount;

        List<Node> nodeList = new List<Node>();
        Node parent = curPointNode;
        float baseAngle = Vector3.Angle(Vector3.right, curPointNode.info.pos - p1)*Mathf.Deg2Rad;
        if ((curPointNode.info.pos - p1).z < 0)
            baseAngle = -baseAngle;

        int rotateRight = rightStart ? -1 : 1;

        for (int i = 1; i < moveCount; i++)
        {
            Vector3 aDir = (p1 - parent.info.pos).normalized;
            float x = p1.x + radius1 * Mathf.Cos(baseAngle + rotateRight * i * angleStep);
            float z = p1.z + radius1 * Mathf.Sin(baseAngle + rotateRight * i * angleStep);
            Vector3 newPosition = new Vector3(x, parent.info.pos.y, z);
            if (rrt.insideObstacle(newPosition))
                return null;
            float xVel = (float)Math.Round((Double)(x - parent.info.pos.x ) / dt, 2, MidpointRounding.AwayFromZero);
            float zVel = (float)Math.Round((Double)(z - parent.info.pos.z) / dt, 2, MidpointRounding.AwayFromZero);
            nodeList.Add(new Node(parent, new pointInfo(newPosition, new Vector3(xVel, 0, zVel), aDir)));
            parent = nodeList[nodeList.Count - 1];
        }
        nodeList.Add(new Node(parent, new pointInfo(tp1, curPointNode.info.vel.magnitude * tangentLine.normalized, tangentLine.normalized)));
        parent = nodeList[nodeList.Count - 1];

        float movingDtsDacc = (goal.vel - parent.info.vel).magnitude / aMax;
        float moveCountDacc = (int)Mathf.Ceil(movingDtsDacc);

        float coefficient = goal.vel.magnitude > parent.info.vel.magnitude ? 1 : -1;
        float dist = 0;
        float vel = parent.info.vel.magnitude;
        for(int i = 0; i < moveCountDacc; i++)
        {
            dist += vel * dt;
            vel = vel + coefficient * aMax * dt;
        }

        if (dist > tangentLine.magnitude)
            return null;
        
        movingDts = (tangentLine.magnitude - dist) / (parent.info.vel.magnitude * dt);
        moveCount = (int)Mathf.Ceil(movingDts);

        // Move on tangent line
        for (int i = 1; i < moveCount; i++)
        {
            Vector3 newPosition = tp1 + i * dt * parent.info.vel.magnitude * tangentLine.normalized;
            if (rrt.insideObstacle(newPosition))
                return null;
            nodeList.Add(new Node(parent, new pointInfo(newPosition, tangentLine.normalized * parent.info.vel.magnitude, tangentLine.normalized)));
            parent = nodeList[nodeList.Count - 1];
        }
        nodeList.Add(new Node(parent, new pointInfo(tp2 - (tangentLine.normalized * dist), tangentLine.normalized * parent.info.vel.magnitude, tangentLine.normalized)));
        parent = nodeList[nodeList.Count - 1];

        for (int i = 1; i < moveCountDacc; i++)
        {
            vel = (parent.info.vel.magnitude + coefficient* aMax) * dt;
            Vector3 newPosition = parent.info.pos + tangentLine.normalized*vel;
            if (rrt.insideObstacle(newPosition))
                return null;
            nodeList.Add(new Node(parent, new pointInfo(newPosition, tangentLine.normalized * vel /dt, tangentLine.normalized)));
            parent = nodeList[nodeList.Count - 1];
        }
        // Add tp2 to list
        nodeList.Add(new Node(parent, new pointInfo(tp2, goal.vel.magnitude * tangentLine.normalized, tangentLine.normalized)));
        parent = nodeList[nodeList.Count - 1];


        arcLength = getArcLength(tp2 - p2, goal.pos - p2, rightGoal, radius2);
        movingDts = arcLength / (dt * goal.vel.magnitude);
        moveCount = (int)Mathf.Ceil(movingDts);
        phi1 = arcLength / radius2;
        angleStep = phi1 / moveCount;
        baseAngle = Vector3.Angle(Vector3.right, tp2 - p2) * Mathf.Deg2Rad;
        if ((tp2 - p2).z < 0)
            baseAngle = -baseAngle;
        rotateRight = rightGoal ? -1 : 1;
        for (int i = 1; i < moveCount; i++)
        {
            Vector3 aDir = (p2 - parent.info.pos).normalized;
            float x = p2.x + radius2 * Mathf.Cos(baseAngle + rotateRight * i * angleStep);
            float z = p2.z + radius2 * Mathf.Sin(baseAngle + rotateRight * i * angleStep);
            Vector3 newPosition = new Vector3(x, parent.info.pos.y, z);
            if (rrt.insideObstacle(newPosition))
                return null;
            float xVel = (float)Math.Round((Double)(x - parent.info.pos.x) / dt, 2, MidpointRounding.AwayFromZero);
            float zVel = (float)Math.Round((Double)(z - parent.info.pos.z) / dt, 2, MidpointRounding.AwayFromZero);
            nodeList.Add(new Node(parent, new pointInfo(newPosition, new Vector3(xVel, 0, zVel), aDir)));
            parent = nodeList[nodeList.Count - 1];
        }

        //Add tp1 to list
        nodeList.Add(new Node(parent, new pointInfo(goal.pos, goal.vel, goal.orientation)));
        return nodeList;
    }


    private float getArcLength(Vector3 v1, Vector3 v2, bool right, float radius)
    {
        float theta = Mathf.Atan2(v1.x, v1.z) - Mathf.Atan2(v2.x, v2.z);
        if (theta < 0f && !right)
            theta = theta + 2 * Mathf.PI;
        else if (theta > 0 && right)
            theta = theta - 2 * Mathf.PI;
        return Mathf.Abs(theta * radius);
    }


    public DynamicPointModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart, RRT rrt)
    : base(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart, rrt)
    {
    }
}