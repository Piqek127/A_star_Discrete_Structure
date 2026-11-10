#include "Algo.h"
namespace
{
    namespace Task1
    {
        int calculateDegree(double matrix[100][100], int node){
            int degree = 0;
            for(int i = 0; i < 100; i++){
                if(matrix[node][i] > 0) ++degree;
            }
            return degree;
        }

        double calculateCost(double matrix[100][100], int u, int v){
            double cost = matrix[u][v] + 0.1*calculateDegree(matrix, v);
            return cost;
        }

        double calculateHeuristic(/* parameters */);
    }

    namespace Task2
    {
        // Helpers exclusively for Task 2
        double calculateEnergy(/* parameters */);

        double calculateHeuristic(/* parameters */);
    }

    namespace Task3
    {
        // Helpers exclusively for Task 3
    }

    namespace Task4
    {
        // Helpers exclusively for Task 4
    }

    namespace Task5
    {
        // Helpers exclusively for Task 5
    }
}


// TODO: Implement Task 1 A* search.
PathNode* findMeshPath(double weightMatrix[100][100], int start, int goal) {
    vector<PathNode*> open;
    vector<PathNode*> closed;

    
    return nullptr;
}

// TODO: Implement Task 2 A* search.
PathNode* findAUVPath(double [100][100], int [100][3], int, int, int) {
    return nullptr;
}

// TODO: Implement Task 3 A* search.
PathNode* findProbabilisticPath(double [100][100], double [100][100],
                                int [100][2], int, int, double, int) {
    return nullptr;
}

// TODO: Implement Task 4 A* search.
PathNode* findGamePath(int [100][100], int, int, int, int, int, int, int) {
    return nullptr;
}

// TODO: Implement Task 5 A* search.
PathNode* findPCBPath(int [100][100], int, int, int, int, int, int,
                      double [100][100], int) {
    return nullptr;
}
