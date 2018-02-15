# DD2438-A1_Path_Planning

This repository includes an implementation of RRT used to solve the path planning problem
for four different vehicle motion models. The RRT implementation differs for each motion
model in the way the nearest node is chosen when a random point is sampled. 

For collision avoidance, a simple raycasting algorithm is implemented (see https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/)
to check whether a point is in a polygon (obstacle). Note that it is assumed that the vehicle has point mass.

The implementation is done in Unity 2017.3.0f3 using C#.
