#ifndef ALGO_H
#define ALGO_H

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
using namespace std;

#include "PathNode.h"



// Task 1 — Mesh Network Packet Routing
PathNode* findMeshPath(double weightMatrix[100][100], int startNode, int goalNode);

// Task 2 — AUV Submarine Navigation in 3D Space
PathNode* findAUVPath(double weightMatrix[100][100], int coords[100][3],
                      int startPoint, int goalPoint, int mode);

// Task 3 — Probabilistic Pathfinding under Hazard Risk
PathNode* findProbabilisticPath(double distMatrix[100][100],
                                double riskMatrix[100][100],
                                int coords[100][2],
                                int startPoint, int goalPoint,
                                double lambda, int mode);

// Task 4 — Game Character Navigation with Multi-Terrain Obstacles
PathNode* findGamePath(int mapGrid[100][100], int m, int n,
                       int startX, int startY, int goalX, int goalY, int mode);

// Task 5 — VLSI PCB Wire Routing
PathNode* findPCBPath(int pcbGrid[100][100], int m, int n,
                      int startX, int startY, int exitX, int exitY,
                      double weightMatrix[100][100], int mode);

#endif
