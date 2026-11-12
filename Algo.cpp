#include "Algo.h"
namespace
{
    namespace Task1
    {

        void calculateDegreeMatrix(double matrix[100][100], int *degree){
            for(int i = 0; i < 100; i++){
                int d = 0;
                for(int j = 0; j < 100; j++){
                    if(matrix[i][j] > 0) ++d;
                }
                degree[i] = d;
            }
        }

        double calculateCost(double matrix[100][100], int u, int v, int* degree){
            double cost = matrix[u][v] + 0.1*degree[v];
            return cost;
        }

        void calculateHeuristicMatrix(double matrix[100][100], double* hScore, int goal, int* degree){
            double minCost = INFINITY;
            vector<int> queue;
            int distance[100] = {};
            bool visited[100] = {false};

            int front = 0;

            fill(distance, distance + 100, -1);
            
            queue.push_back(goal);
            distance[goal] = 0;
            visited[goal] = true;

            for (int i = 0; i < 100; i++){
                for(int j = 0; j < 100; j ++){
                    if(matrix[i][j] > 0){
                        double cost = calculateCost(matrix, i, j, degree);
                        if (cost < minCost){
                            minCost = cost;
                        }
                    }
                }
            }


            //BFS to find the minimun edges from goal to every node
            while(front < queue.size()){
                int current = queue[front];
                front++;

                for(int i = 0; i < 100; i++){
                    if (matrix[i][current] > 0 && !visited[i]){
                        queue.push_back(i);
                        visited[i] = true;
                        distance[i] = distance[current] + 1;
                    }
                }

            }

            if(minCost == INFINITY){
                fill(hScore, hScore + 100, 0);
                return;
            }

            for(int i = 0 ; i < 100;  i++){
                if(distance[i] == -1){
                    hScore[i] = 0;
                }
                else{
                    /*
                    Calculate heuristic optimistically by multiplying the minimum cost 
                    between 2 nodes and the shortest path from the current node to goal
                    */
                    hScore[i] = distance[i] * minCost;
                }
            }
        }

        
    }

    namespace Task2
    {
        // Helpers exclusively for Task 2
        double calculateEnergy(double matrix[100][100], int coords[100][3], double gScore[100], int u, int v){
            double depthDelta = abs(coords[v][2] - coords[u][2]);
            return gScore[u] + matrix[u][v]*(1+ depthDelta/100);
        }

        double heuristicDistance(int coords[100][3], int goal, int node, int mode){
            double dx = abs(coords[node][0] - coords[goal][0]);
            double dy = abs(coords[node][1] - coords[goal][1]);
            double dz = abs(coords[node][2] - coords[goal][2]);

            switch (mode){
                /*
                Notice what we are calculating is energy while  the heuristics are based 
                on coordinates(distance). This can cause the heuristic to over calculate, following
                Manhattan ≥ Euclidean ≥ Chebyshev, if the weight and distance are weird enough.
                */
                case 1: return dx + dy + dz;
                case 2: return sqrt(dx*dx + dy*dy + dz*dz);
                case 3: return max(dx, max(dy, dz));
                default: return 0;
            }
        }

        void calculateHeuristicMatrix(int coords[100][3], double *hScore, int goal, int mode){
            for (int i = 0; i < 100; i++){
                hScore[i] = heuristicDistance(coords, goal, i, mode);
            }
        }
    }

    namespace Task3
    {
        // Helpers exclusively for Task 3
        double calculateCost(double weightMatrix[100][100], double riskMatrix[100][100], double gScore[100], double penaltyFator , int u, int v){
            return gScore[u] + (weightMatrix[u][v] + penaltyFator*riskMatrix[u][v]);
        }

        double calculatePSafe(double riskMatrix[100][100], double pSafe[100], int u, int v){
            return pSafe[u]*(1-riskMatrix[u][v]);
        }

        double heuristicDistance(int coords[100][2], int goal, int node, int mode){
            double dx = abs(coords[node][0] - coords[goal][0]);
            double dy = abs(coords[node][1] - coords[goal][1]);

            switch (mode){
                case 1: return dx + dy;
                case 2: return sqrt(dx*dx + dy*dy);
                default: return 0;
            }
        }

        void calculateHeuristicMatrix(int coords[100][2], double *hScore, int goal, int mode){
            for (int i = 0; i < 100; i++){
                hScore[i] = heuristicDistance(coords, goal, i, mode);
            }
        }
    }

    namespace Task4
    {
        // Helpers exclusively for Task 4
        double heuristicDistance(int coords[100][2], int goal, int node, int mode){
            double dx = abs(coords[node][0] - coords[goal][0]);
            double dy = abs(coords[node][1] - coords[goal][1]);

            switch (mode){
                case 1: return dx + dy;
                case 2: return sqrt(dx*dx + dy*dy);
                default: return 0;
            }
        }

        void calculateHeuristicMatrix(int coords[100][2], double *hScore, int goal, int mode){
            for (int i = 0; i < 100; i++){
                hScore[i] = heuristicDistance(coords, goal, i, mode);
            }
        }
    }

    namespace Task5
    {
        // Helpers exclusively for Task 5
    }
}


// TODO: Implement Task 1 A* search.
PathNode* findMeshPath(double weightMatrix[100][100], int start, int goal) {
    double gScore[100];
    double hScore[100];
    double fScore[100];
    int degree[100];
    int parents[100];
    int current = -1;
    vector<int> open = {start};

    std::fill(gScore, gScore + 100, INFINITY);
    std::fill(hScore, hScore + 100, 0);
    std::fill(fScore, fScore + 100, INFINITY);
    std::fill(parents, parents + 100, -1);

    Task1::calculateDegreeMatrix(weightMatrix, degree);
    Task1::calculateHeuristicMatrix(weightMatrix, hScore, goal, degree);

    gScore[start] = 0;
    fScore[start] = 0 + hScore[start];
    

    while (!open.empty()){
        double minfScore = INFINITY;
        current = -1;
        for(size_t i = 0; i < open.size(); i++){
            if(fScore[open[i]] < minfScore){
                current = open[i];
                minfScore = fScore[open[i]];
            }
        }

        if (current == -1){
            break;
        }

        open.erase(
            find(open.begin(), open.end(), current)
        );
        
        if (current == goal){
            break;
        }

        // Check each node realated to the current one
        for(int i = 0; i < 100; i ++){

            if (weightMatrix[current][i] > 0){
                double newG = gScore[current] + Task1::calculateCost(weightMatrix, current, i, degree);
                //Check if gScore has been calculated and if that calculated value needs updating
                if(gScore[i] > newG){
                    if (find(open.begin(), open.end(), i) == open.end())
                        open.push_back(i);
                    gScore[i] = newG;
                    fScore[i] = gScore[i] + hScore[i];
                    parents[i] = current;

                }
        
            }
        }
    }

    if (current != goal) {
    return nullptr;
}

    PathNode* head = nullptr;
    while (true) {
        PathNode* temp = createPathNode(to_string(current), fScore[current], gScore[current], hScore[current]);
        temp->next = head;
        head = temp;
        if (current == start) break;
        current = parents[current];
        if (current == -1) return nullptr;
    }


    return head;
}

// TODO: Implement Task 2 A* search.
PathNode* findAUVPath(double weightMatrix[100][100], int coords[100][3], int start, int goal, int mode) {
    double gScore[100];
    double hScore[100];
    double fScore[100];
    int parents[100];
    int current = -1;
    vector<int> open = {start};

    std::fill(gScore, gScore + 100, INFINITY);
    std::fill(hScore, hScore + 100, 0);
    std::fill(fScore, fScore + 100, INFINITY);
    std::fill(parents, parents + 100, -1);

    Task2::calculateHeuristicMatrix(coords, hScore, goal, mode);

    gScore[start] = 0;
    fScore[start] = 0 + hScore[start];
    

    while (!open.empty()){
        double minfScore = INFINITY;
        current = -1;
        for(size_t i = 0; i < open.size(); i++){
            if(fScore[open[i]] < minfScore){
                current = open[i];
                minfScore = fScore[open[i]];
            }
        }

        if (current == -1)
            break;

        open.erase(
            find(open.begin(), open.end(), current)
        );
        
        if (current == goal){
            break;
        }

        // Check each node realated to the current one
        for(int i = 0; i < 100; i ++){
            
            if (weightMatrix[current][i] > 0){
                double newG = Task2::calculateEnergy(weightMatrix, coords, gScore, current, i);
                //Check if gScore has been calculated and if that calculated value needs updating
                if(gScore[i] > newG){
                    if (find(open.begin(), open.end(), i) == open.end())
                        open.push_back(i);
                    gScore[i] = newG;
                    fScore[i] = gScore[i] + hScore[i];
                    parents[i] = current;

                }
        
            }
        }
    }

    if (current != goal) {
    return nullptr;
}

    PathNode* head = nullptr;
    while (true) {
        PathNode* temp = createPathNode(to_string(current), fScore[current], gScore[current], hScore[current]);
        temp->next = head;
        head = temp;
        if (current == start) break;
        current = parents[current];
        if (current == -1) return nullptr;
    }


    return head;
}

// TODO: Implement Task 3 A* search.
PathNode* findProbabilisticPath(double weightMatrix[100][100], double riskMatrix[100][100],
                                int coords[100][2], int start, int goal, double lambda, int mode) {
    double gScore[100];
    double hScore[100];
    double fScore[100];
    double pSafe[100];
    int parents[100];
    int current = -1;
    vector<int> open = {start};

    std::fill(gScore, gScore + 100, INFINITY);
    std::fill(hScore, hScore + 100, 0);
    std::fill(fScore, fScore + 100, INFINITY);
    std::fill(pSafe, pSafe + 100, 0);
    std::fill(parents, parents + 100, -1);

    Task3::calculateHeuristicMatrix(coords, hScore, goal, mode);

    gScore[start] = 0;
    pSafe[start] = 1;                                
    fScore[start] = 0 + hScore[start];
    

    while (!open.empty()){
        double minfScore = INFINITY;
        current = -1;
        for(size_t i = 0; i < open.size(); i++){
            if(fScore[open[i]] < minfScore){
                current = open[i];
                minfScore = fScore[open[i]];
            }
        }

        if (current == -1)
            break;

        open.erase(
            find(open.begin(), open.end(), current)
        );
        
        if (current == goal){
            break;
        }

        // Check each node realated to the current one
        for(int i = 0; i < 100; i ++){
            
            if (weightMatrix[current][i] > 0){
                double newG = Task3::calculateCost(weightMatrix, riskMatrix, gScore, lambda, current, i);
                //Check if gScore has been calculated and if that calculated value needs updating
                if(gScore[i] > newG){
                    if (find(open.begin(), open.end(), i) == open.end())
                        open.push_back(i);
                    pSafe[i] = Task3::calculatePSafe(riskMatrix, pSafe, current, i);
                    gScore[i] = newG;
                    fScore[i] = gScore[i] + hScore[i];
                    parents[i] = current;

                }
        
            }
        }
    }

    if (current != goal) {
    return nullptr;
}

    PathNode* head = nullptr;
    while (true) {
        PathNode* temp = createPathNode(to_string(current), fScore[current], gScore[current], hScore[current]);
        temp->next = head;
        head = temp;
        if (current == start) break;
        current = parents[current];
        if (current == -1) return nullptr;
    }


    return head;
}

// TODO: Implement Task 4 A* search.
PathNode* findGamePath(int mapGrid[100][100], int m, int n, int startX, int startY, int goalX, int goalY, int mode) {
    double gScore[100];
    double hScore[100];
    double fScore[100];
    double pSafe[100];
    int parents[100];
    int current = -1;
    vector<int> open = {start};

    std::fill(gScore, gScore + 100, INFINITY);
    std::fill(hScore, hScore + 100, 0);
    std::fill(fScore, fScore + 100, INFINITY);
    std::fill(pSafe, pSafe + 100, 0);
    std::fill(parents, parents + 100, -1);

    Task3::calculateHeuristicMatrix(coords, hScore, goal, mode);

    gScore[start] = 0;
    pSafe[start] = 1;                                
    fScore[start] = 0 + hScore[start];
    

    while (!open.empty()){
        double minfScore = INFINITY;
        current = -1;
        for(size_t i = 0; i < open.size(); i++){
            if(fScore[open[i]] < minfScore){
                current = open[i];
                minfScore = fScore[open[i]];
            }
        }

        if (current == -1)
            break;

        open.erase(
            find(open.begin(), open.end(), current)
        );
        
        if (current == goal){
            break;
        }

        // Check each node realated to the current one
        for(int i = 0; i < 100; i ++){
            
            if (weightMatrix[current][i] > 0){
                double newG = Task3::calculateCost(weightMatrix, riskMatrix, gScore, lambda, current, i);
                //Check if gScore has been calculated and if that calculated value needs updating
                if(gScore[i] > newG){
                    if (find(open.begin(), open.end(), i) == open.end())
                        open.push_back(i);
                    pSafe[i] = Task3::calculatePSafe(riskMatrix, pSafe, current, i);
                    gScore[i] = newG;
                    fScore[i] = gScore[i] + hScore[i];
                    parents[i] = current;

                }
        
            }
        }
    }

    if (current != goal) {
    return nullptr;
}

    PathNode* head = nullptr;
    while (true) {
        PathNode* temp = createPathNode(to_string(current), fScore[current], gScore[current], hScore[current]);
        temp->next = head;
        head = temp;
        if (current == start) break;
        current = parents[current];
        if (current == -1) return nullptr;
    }


    return head;
}

// TODO: Implement Task 5 A* search.
PathNode* findPCBPath(int [100][100], int, int, int, int, int, int,
                      double [100][100], int) {
    return nullptr;
}
