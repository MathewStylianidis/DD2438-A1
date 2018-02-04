using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

public class KinematicDriveModel : BaseModel
{
    public int contendersCount = 10;

    public override pointInfo moveTowards(pointInfo curPointInfo, Vector3 targetPos)
    {
        Vector3 path = targetPos - curPointInfo.pos;
        float angle = Vector3.Angle(curPointInfo.orientation, path);
        float orientation = Vector3.Angle(Vector3.right, curPointInfo.orientation);
        if (curPointInfo.orientation.z < 0)
            orientation = -orientation;
        float phi = 0;
        if (angle > 0.00001)
        {
            Vector3 cross = Vector3.Cross(path, curPointInfo.orientation);
            float partTurn = (((vMax / length) * Mathf.Tan(phiMax)) * Mathf.Rad2Deg * dt) / angle;
            if (partTurn <= 1.0f)
                phi = phiMax * Mathf.Sign(cross.y);
            else
                phi = Mathf.Atan((angle * Mathf.Deg2Rad * length / vMax)) * Mathf.Sign(cross.y);
        }
        float deltaTheta = (((vMax / length) * Mathf.Tan(phi)) * dt);
        float newAngle = deltaTheta + orientation * Mathf.Deg2Rad;
        Vector3 newOrientation = new Vector3(Mathf.Cos(newAngle), 0, Mathf.Sin(newAngle));
        float xMove = vMax * Mathf.Cos(newAngle) * dt;
        float yMove = vMax * Mathf.Sin(newAngle) * dt;
        Vector3 newPosition = new Vector3(curPointInfo.pos.x + xMove, curPointInfo.pos.y, curPointInfo.pos.z + yMove);

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
        float radius = length / Mathf.Tan(phiMax);
        List<Node> rl = RL(curPointNode, goal, radius);
        //List<Node> ll = LL(curPointNode, goal, radius);
     
        return rl;
    }

    private List<Node> dubinPath(Node curPointNode, pointInfo goal, float radius, Vector3 tp1, Vector3 tp2, Vector3 p1, Vector3 p2, bool rightStart, bool rightGoal)
    {
        Vector3 tangentLine = tp2 - tp1;

        if (rrt.insideObstacle(tp1) || rrt.insideObstacle(tp2))
            return null;
        float arcLength = getArcLength(curPointNode.info.pos - p1, tp1 - p1, rightStart, radius);

        float movingDts = arcLength / (dt * vMax);
        int moveCount = (int)Mathf.Ceil(movingDts);

        // get current orientation
        float orientation = Vector3.Angle(Vector3.right, curPointNode.info.orientation);
        if (curPointNode.info.orientation.z < 0)
            orientation = -orientation;

        float deltaTheta = (((vMax / length) * Mathf.Tan(phiMax)) * dt) * Mathf.Rad2Deg;
        List<Node> nodeList = new List<Node>();
        Node parent = curPointNode;
        int coefficient = rightStart ? -1 : 1;
        for (int i = 1; i < moveCount; i++)
        {
            float newAngle = (orientation + coefficient * i * deltaTheta) * Mathf.Deg2Rad;
            Vector3 newOrientation = new Vector3(Mathf.Cos(newAngle), 0, Mathf.Sin(newAngle));
            float xMove = vMax * Mathf.Cos(newAngle) * dt;
            float yMove = vMax * Mathf.Sin(newAngle) * dt;
            Vector3 newPosition = new Vector3(parent.info.pos.x + xMove, parent.info.pos.y, parent.info.pos.z + yMove);
            if (rrt.insideObstacle(newPosition))
                return null;
            float xVel = (float)Math.Round((Double)xMove / dt, 2, MidpointRounding.AwayFromZero);
            float zVel = (float)Math.Round((Double)yMove / dt, 2, MidpointRounding.AwayFromZero);
            nodeList.Add(new Node(parent, new pointInfo(newPosition, new Vector3(xVel, 0, zVel), newOrientation)));
            parent = nodeList[nodeList.Count - 1];
        }
        //Add tp1 to list
        float tp1Velx = (float)Math.Round((Double)(tp1.x - parent.info.pos.x) / dt, 2, MidpointRounding.AwayFromZero);
        float tp1Velz = (float)Math.Round((Double)(tp1.z - parent.info.pos.z) / dt, 2, MidpointRounding.AwayFromZero);
        nodeList.Add(new Node(parent, new pointInfo(tp1, new Vector3(tp1Velx, 0f, tp1Velz), tangentLine.normalized)));
        parent = nodeList[nodeList.Count - 1];

        movingDts = Vector3.Magnitude(tangentLine) / (vMax * dt);
        moveCount = (int)Mathf.Ceil(movingDts);
        // Move on tangent line
        for (int i = 1; i < moveCount; i++)
        {
            Vector3 newPosition = tp1 + i * dt * vMax * tangentLine.normalized;
            if (rrt.insideObstacle(newPosition))
                return null;
            nodeList.Add(new Node(parent, new pointInfo(newPosition, tangentLine.normalized * vMax, tangentLine.normalized)));
            parent = nodeList[nodeList.Count - 1];
        }
        // Add tp2 to list
        float tp2Velx = (float)Math.Round((Double)(tp2.x - parent.info.pos.x) / dt, 2, MidpointRounding.AwayFromZero);
        float tp2Velz = (float)Math.Round((Double)(tp2.z - parent.info.pos.z) / dt, 2, MidpointRounding.AwayFromZero);
        nodeList.Add(new Node(parent, new pointInfo(tp2, new Vector3(tp2Velx, 0f, tp2Velz), tangentLine.normalized)));
        parent = nodeList[nodeList.Count - 1];


        arcLength = getArcLength(tp2 - p2, goal.pos - p2, rightGoal, radius);
        movingDts = arcLength / (dt * vMax);
        moveCount = (int)Mathf.Ceil(movingDts);

        // get current orientation
        orientation = Vector3.Angle(Vector3.right, tangentLine.normalized);
        if (tangentLine.normalized.z < 0)
            orientation = -orientation;

        deltaTheta = (((vMax / length) * Mathf.Tan(phiMax)) * dt) * Mathf.Rad2Deg;
        coefficient = rightGoal ? -1 : 1;
        for (int i = 1; i < moveCount; i++)
        {
            //Always decrease the orientation because we always move right on the R circle
            float newAngle = (orientation + coefficient * i * deltaTheta) * Mathf.Deg2Rad;
            Vector3 newOrientation = new Vector3(Mathf.Cos(newAngle), 0, Mathf.Sin(newAngle));
            float xMove = vMax * Mathf.Cos(newAngle) * dt;
            float yMove = vMax * Mathf.Sin(newAngle) * dt;
            Vector3 newPosition = new Vector3(parent.info.pos.x + xMove, parent.info.pos.y, parent.info.pos.z + yMove);
            if (rrt.insideObstacle(newPosition))
                return null;
            float xVel = (float)Math.Round((Double)xMove / dt, 2, MidpointRounding.AwayFromZero);
            float zVel = (float)Math.Round((Double)yMove / dt, 2, MidpointRounding.AwayFromZero);
            nodeList.Add(new Node(parent, new pointInfo(newPosition, new Vector3(xVel, 0, zVel), newOrientation)));
            parent = nodeList[nodeList.Count - 1];
        }
        //Add tp1 to list
        nodeList.Add(new Node(parent, new pointInfo(goal.pos, goal.vel, goal.orientation)));
        return nodeList;

    }

    private List<Node> LR(Node curPointNode, pointInfo goal, float radius)
    {
        Vector3 dir = Vector3.Cross(Vector3.up, curPointNode.info.vel).normalized;
        Vector3 p1 = curPointNode.info.pos - dir * radius;
        Vector3 dir2 = Vector3.Cross(Vector3.up, goal.vel).normalized;
        Vector3 p2 = goal.pos + dir2 * radius;
        Vector3 D = p2 - p1;
        float angle = -Mathf.Acos(radius / (D.magnitude / 2));
        float baseAngle = Vector3.Angle(Vector3.right, D) * Mathf.Deg2Rad;
        if (D.z < 0)
            baseAngle = -baseAngle;
        Vector3 tp1 = p1 + new Vector3(Mathf.Cos(angle + baseAngle), 0, Mathf.Sin(angle + baseAngle)) * radius;
        Vector3 tp2 = p2 - new Vector3(Mathf.Cos(angle + baseAngle), 0, Mathf.Sin(angle + baseAngle)) * radius;

        return dubinPath(curPointNode, goal, radius, tp1, tp2, p1, p2, false, true);
    }


    private List<Node> RL(Node curPointNode, pointInfo goal, float radius)
    {
        Vector3 dir = Vector3.Cross(Vector3.up, curPointNode.info.vel).normalized;
        Vector3 p1 = curPointNode.info.pos + dir * radius;
        Vector3 dir2 = Vector3.Cross(Vector3.up, goal.vel).normalized;
        Vector3 p2 = goal.pos - dir2 * radius;
        Vector3 D = p2 - p1;
        float angle = Mathf.Acos(radius / (D.magnitude / 2));
        float baseAngle = Vector3.Angle(Vector3.right, D)*Mathf.Deg2Rad;
        if (D.z < 0)
            baseAngle = -baseAngle;
        Vector3 tp1 = p1 + new Vector3(Mathf.Cos(angle+ baseAngle), 0, Mathf.Sin(angle+ baseAngle))*radius;
        Vector3 tp2 = p2 - new Vector3(Mathf.Cos(angle+ baseAngle), 0, Mathf.Sin(angle+ baseAngle)) * radius;

        return dubinPath(curPointNode, goal, radius, tp1, tp2, p1, p2, true, false);
    }

    private List<Node> LL(Node curPointNode, pointInfo goal, float radius)
    {
        Vector3 dir = Vector3.Cross(Vector3.up, curPointNode.info.vel).normalized;
        Vector3 p1 = curPointNode.info.pos - dir * radius;
        Vector3 dir2 = Vector3.Cross(Vector3.up, goal.vel).normalized;
        Vector3 p2 = goal.pos - dir2 * radius;

        Vector3 D = p2 - p1;
        Vector3 tpDir1 = Vector3.Cross(D, Vector3.up).normalized;
        Vector3 tp1 = p1 - tpDir1 * radius;
        Vector3 tp2 = p2 - tpDir1 * radius;

        return dubinPath(curPointNode, goal, radius, tp1, tp2, p1, p2, false, false);
    }


    private List<Node> RR(Node curPointNode, pointInfo goal, float radius)
    {
        Vector3 dir = Vector3.Cross(Vector3.up, curPointNode.info.vel).normalized;
        Vector3 p1 = curPointNode.info.pos + dir * radius;
        Vector3 dir2 = Vector3.Cross(Vector3.up, goal.vel).normalized;
        Vector3 p2 = goal.pos + dir2 * radius;

        Vector3 D = p2 - p1;
        Vector3 tpDir1 = Vector3.Cross(D, Vector3.up).normalized;
        Vector3 tp1 = p1 + tpDir1 * radius;
        Vector3 tp2 = p2 + tpDir1 * radius;


        return dubinPath(curPointNode, goal, radius, tp1, tp2, p1, p2, true, true);
    }

    private float getArcLength(Vector3 v1, Vector3 v2, bool right, float radius)
    {
        float theta = Mathf.Atan2(v1.x, v1.z) - Mathf.Atan2(v2.x,v2.z);
        if (theta < 0f && !right)
            theta = theta + 2 * Mathf.PI;
        else if (theta > 0 && right)
            theta = theta - 2 * Mathf.PI;
        return Mathf.Abs(theta * radius);
    }

    public KinematicDriveModel(Vector3 posGoal, Vector3 posStart, float length, float aMax, float dt, float omegaMax, float phiMax, float t, float vMax, Vector3 velGoal, Vector3 velStart, RRT rrt)
        : base(posGoal, posStart, length, aMax, dt, omegaMax, phiMax, t, vMax, velGoal, velStart, rrt)
    {
    }
}