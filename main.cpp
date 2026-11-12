#include "PathNode.h"
#include "Algo.h"


namespace
{
    int passed = 0;
    int failed = 0;
    int skipped = 0;

    void check(bool condition, const string& label)
    {
        if (condition)
        {
            ++passed;
            cout << "  [PASS] " << label << "\n";
        }
        else
        {
            ++failed;
            cout << "  [FAIL] " << label << "\n";
        }
    }

    bool nearlyEqual(double a, double b)
    {
        return fabs(a - b) < 1e-9;
    }

    void printPath(PathNode* head)
    {
        cout << "Solution Path:\n";
        while (head != nullptr)
        {
            cout << "Node: " << head->name
                 << " | f: " << head->f
                 << " | g: " << head->g
                 << " | h: " << head->h << "\n";
            head = head->next;
        }
    }

    int pathLength(PathNode* head)
    {
        int count = 0;
        for (; head != nullptr; head = head->next) ++count;
        return count;
    }

    PathNode* lastNode(PathNode* head)
    {
        while (head != nullptr && head->next != nullptr) head = head->next;
        return head;
    }

    void clearMatrix(double matrix[100][100])
    {
        for (int i = 0; i < 100; i++)
            for (int j = 0; j < 100; j++)
                matrix[i][j] = 0;
    }

    void addEdge(double matrix[100][100], int u, int v, double weight)
    {
        matrix[u][v] = weight;
        matrix[v][u] = weight;
    }

    void generateWeightedMatrix(double matrix[100][100], int numNodes)
    {
        clearMatrix(matrix);
        for (int i = 0; i < numNodes; i++)
        {
            for (int j = i + 1; j < numNodes; j++)
            {
                if ((i + j) % 3 != 0)
                {
                    double weight = ((i + 1) * (j + 2)) % 20 + 1;
                    addEdge(matrix, i, j, weight);
                }
            }
        }
    }

    void printMatrix(double matrix[100][100], int numNodes)
    {
        for (int i = 0; i < numNodes; i++)
        {
            for (int j = 0; j < numNodes; j++)
                cout << matrix[i][j] << "\t";
            cout << '\n';
        }
    }

    namespace Common
    {
        void run()
        {
            PathNode* a = createPathNode("A", 4.0, 0.0, 4.0);
            PathNode* b = createPathNode("B", 3.0, 1.0, 2.0);
            PathNode* c = createPathNode("C", 4.0, 4.0, 0.0);
            a->next = b;
            b->next = c;

            check(pathLength(a) == 3, "linked list holds three nodes");
            check(lastNode(a) == c, "last node is the goal node");
            check(a->name == "A" && nearlyEqual(a->f, 4.0), "node fields are stored");

            deletePath(a);
            deletePath(nullptr);
            check(true, "deletePath accepts nullptr");
        }
    }

    namespace Task1
    {
        void degrees(double matrix[100][100], int* degree)
        {
            for (int i = 0; i < 100; i++)
            {
                degree[i] = 0;
                for (int j = 0; j < 100; j++)
                    if (matrix[i][j] > 0) ++degree[i];
            }
        }

        double referenceCost(double matrix[100][100], int start, int goal)
        {
            int degree[100];
            double dist[100];
            bool done[100];
            degrees(matrix, degree);
            fill(dist, dist + 100, (double)INFINITY);
            fill(done, done + 100, false);
            dist[start] = 0;

            for (int step = 0; step < 100; step++)
            {
                int u = -1;
                for (int i = 0; i < 100; i++)
                    if (!done[i] && dist[i] < INFINITY && (u == -1 || dist[i] < dist[u]))
                        u = i;
                if (u == -1) break;
                done[u] = true;

                for (int v = 0; v < 100; v++)
                {
                    if (matrix[u][v] > 0)
                    {
                        double candidate = dist[u] + matrix[u][v] + 0.1 * degree[v];
                        if (candidate < dist[v]) dist[v] = candidate;
                    }
                }
            }
            return dist[goal];
        }

        bool validatePath(double matrix[100][100], PathNode* head, int start, int goal)
        {
            if (head == nullptr) return false;

            int degree[100];
            degrees(matrix, degree);

            if (head->name != to_string(start)) return false;
            if (!nearlyEqual(head->g, 0.0)) return false;
            if (!nearlyEqual(head->f, head->g + head->h)) return false;

            for (PathNode* cur = head; cur->next != nullptr; cur = cur->next)
            {
                PathNode* nxt = cur->next;
                int u = stoi(cur->name);
                int v = stoi(nxt->name);
                if (matrix[u][v] <= 0) return false;
                double expectedG = cur->g + matrix[u][v] + 0.1 * degree[v];
                if (!nearlyEqual(nxt->g, expectedG)) return false;
                if (!nearlyEqual(nxt->f, nxt->g + nxt->h)) return false;
            }

            PathNode* tail = lastNode(head);
            return tail->name == to_string(goal) && nearlyEqual(tail->h, 0.0);
        }

        void testSamePoint()
        {
            double matrix[100][100];
            generateWeightedMatrix(matrix, 6);

            PathNode* path = findMeshPath(matrix, 2, 2);
            check(path != nullptr, "start == goal returns a path");
            check(pathLength(path) == 1, "start == goal path has exactly one node");
            if (path != nullptr)
                check(path->name == "2" && nearlyEqual(path->g, 0.0) && nearlyEqual(path->f, 0.0),
                      "start == goal node has g = 0 and f = 0");
            deletePath(path);
        }

        void testDirectEdge()
        {
            double matrix[100][100];
            clearMatrix(matrix);
            addEdge(matrix, 0, 1, 5);

            PathNode* path = findMeshPath(matrix, 0, 1);
            check(pathLength(path) == 2, "single edge graph gives a two node path");
            if (path != nullptr)
            {
                PathNode* tail = lastNode(path);
                check(nearlyEqual(tail->g, 5.0 + 0.1 * 1), "g = weight + 0.1 * degree(goal)");
                check(validatePath(matrix, path, 0, 1), "path fields are consistent");
            }
            deletePath(path);
        }

        void testDegreePenalty()
        {
            double matrix[100][100];
            clearMatrix(matrix);

            addEdge(matrix, 0, 1, 1.0);
            addEdge(matrix, 1, 4, 1.0);
            addEdge(matrix, 0, 2, 1.4);
            addEdge(matrix, 2, 4, 1.4);
            for (int leaf = 5; leaf < 15; leaf++)
                addEdge(matrix, 1, leaf, 5.0);

            PathNode* path = findMeshPath(matrix, 0, 4);
            printPath(path);

            check(pathLength(path) == 3, "path has three nodes");
            if (path != nullptr)
            {
                check(path->next != nullptr && path->next->name == "2",
                      "avoids high degree hub 1 even though its raw weights are lower");
                check(nearlyEqual(lastNode(path)->g, 3.2), "total cost is 3.2");
                check(validatePath(matrix, path, 0, 4), "path fields are consistent");
            }
            deletePath(path);
        }

        void testUnreachable()
        {
            double matrix[100][100];
            clearMatrix(matrix);
            addEdge(matrix, 0, 1, 2);
            addEdge(matrix, 2, 3, 2);

            PathNode* path = findMeshPath(matrix, 0, 3);
            check(path == nullptr, "goal in another component returns nullptr");
            deletePath(path);

            path = findMeshPath(matrix, 0, 99);
            check(path == nullptr, "goal with no edges returns nullptr");
            deletePath(path);
        }

        void testAllPairs(int numNodes)
        {
            double matrix[100][100];
            generateWeightedMatrix(matrix, numNodes);

            int mismatches = 0;
            int checkedPairs = 0;
            for (int s = 0; s < numNodes; s++)
            {
                for (int t = 0; t < numNodes; t++)
                {
                    double expected = referenceCost(matrix, s, t);
                    PathNode* path = findMeshPath(matrix, s, t);
                    ++checkedPairs;

                    if (expected == INFINITY)
                    {
                        if (path != nullptr) ++mismatches;
                    }
                    else if (path == nullptr || !validatePath(matrix, path, s, t) ||
                             !nearlyEqual(lastNode(path)->g, expected))
                    {
                        ++mismatches;
                        cout << "    mismatch for " << s << " -> " << t << "\n";
                    }
                    deletePath(path);
                }
            }
            check(mismatches == 0, "all " + to_string(checkedPairs) + " pairs match Dijkstra on "
                                       + to_string(numNodes) + " generated nodes");
        }

        void testGeneratedSample()
        {
            double matrix[100][100];
            generateWeightedMatrix(matrix, 6);
            cout << "Generated weighted matrix:\n";
            printMatrix(matrix, 6);

            PathNode* path = findMeshPath(matrix, 0, 1);
            printPath(path);
            check(path != nullptr, "sample 0 -> 1 returns a path");
            deletePath(path);
        }

        void testEmptyGraph()
        {
            double matrix[100][100];
            clearMatrix(matrix);

            PathNode* path = findMeshPath(matrix, 3, 3);
            check(pathLength(path) == 1, "empty graph: start == goal still returns one node");
            deletePath(path);

            path = findMeshPath(matrix, 3, 4);
            check(path == nullptr, "empty graph: different goal returns nullptr");
            deletePath(path);
        }

        void run()
        {
            testSamePoint();
            testDirectEdge();
            testDegreePenalty();
            testUnreachable();
            testEmptyGraph();
            testGeneratedSample();
            testAllPairs(6);
            testAllPairs(15);
            testAllPairs(40);
        }
    }

    namespace Task2
    {
        double depthFactor(int z1, int z2)
        {
            return 1.0 + abs(z1 - z2) / 100.0;
        }

        double distanceTo(int coords[100][3], int node, int goal, int mode)
        {
            double dx = abs(coords[node][0] - coords[goal][0]);
            double dy = abs(coords[node][1] - coords[goal][1]);
            double dz = abs(coords[node][2] - coords[goal][2]);
            if (mode == 1) return dx + dy + dz;
            if (mode == 2) return sqrt(dx * dx + dy * dy + dz * dz);
            return max(dx, max(dy, dz));
        }

        void setCoord(int coords[100][3], int i, int x, int y, int z)
        {
            coords[i][0] = x;
            coords[i][1] = y;
            coords[i][2] = z;
        }

        void clearCoords(int coords[100][3])
        {
            for (int i = 0; i < 100; i++)
                setCoord(coords, i, 0, 0, 0);
        }

        void generateGeometricGraph(double matrix[100][100], int coords[100][3], int numNodes)
        {
            clearMatrix(matrix);
            clearCoords(coords);
            for (int i = 0; i < numNodes; i++)
                setCoord(coords, i, (i * 37) % 50, (i * 17) % 50, (i * 29) % 60);

            for (int i = 0; i < numNodes; i++)
            {
                for (int j = i + 1; j < numNodes; j++)
                {
                    if ((i * 7 + j) % 4 != 0)
                    {
                        int manhattan = abs(coords[i][0] - coords[j][0])
                                      + abs(coords[i][1] - coords[j][1])
                                      + abs(coords[i][2] - coords[j][2]);
                        addEdge(matrix, i, j, manhattan + (i * j) % 7 + 1);
                    }
                }
            }
        }

        double referenceCost(double matrix[100][100], int coords[100][3], int start, int goal)
        {
            double dist[100];
            bool done[100];
            fill(dist, dist + 100, (double)INFINITY);
            fill(done, done + 100, false);
            dist[start] = 0;

            for (int step = 0; step < 100; step++)
            {
                int u = -1;
                for (int i = 0; i < 100; i++)
                    if (!done[i] && dist[i] < INFINITY && (u == -1 || dist[i] < dist[u]))
                        u = i;
                if (u == -1) break;
                done[u] = true;

                for (int v = 0; v < 100; v++)
                {
                    if (matrix[u][v] > 0)
                    {
                        double candidate = dist[u] + matrix[u][v] * depthFactor(coords[u][2], coords[v][2]);
                        if (candidate < dist[v]) dist[v] = candidate;
                    }
                }
            }
            return dist[goal];
        }

        bool validatePath(double matrix[100][100], int coords[100][3], PathNode* head,
                          int start, int goal, int mode)
        {
            if (head == nullptr) return false;
            if (head->name != to_string(start)) return false;
            if (!nearlyEqual(head->g, 0.0)) return false;

            for (PathNode* cur = head; cur != nullptr; cur = cur->next)
            {
                int u = stoi(cur->name);
                if (!nearlyEqual(cur->h, distanceTo(coords, u, goal, mode))) return false;
                if (!nearlyEqual(cur->f, cur->g + cur->h)) return false;

                if (cur->next != nullptr)
                {
                    int v = stoi(cur->next->name);
                    if (matrix[u][v] <= 0) return false;
                    double expectedG = cur->g + matrix[u][v] * depthFactor(coords[u][2], coords[v][2]);
                    if (!nearlyEqual(cur->next->g, expectedG)) return false;
                }
            }

            PathNode* tail = lastNode(head);
            return tail->name == to_string(goal) && nearlyEqual(tail->h, 0.0);
        }

        double finalCost(PathNode* path)
        {
            PathNode* tail = lastNode(path);
            return tail == nullptr ? -1.0 : tail->g;
        }

        string describe(PathNode* path)
        {
            string text;
            for (PathNode* cur = path; cur != nullptr; cur = cur->next)
                text += (cur == path ? "" : " -> ") + cur->name;
            return text;
        }

        void testSamePoint()
        {
            double matrix[100][100];
            int coords[100][3];
            clearMatrix(matrix);
            clearCoords(coords);
            addEdge(matrix, 0, 1, 4);
            setCoord(coords, 1, 3, 3, 3);

            for (int mode = 1; mode <= 3; mode++)
            {
                PathNode* path = findAUVPath(matrix, coords, 1, 1, mode);
                check(pathLength(path) == 1 && validatePath(matrix, coords, path, 1, 1, mode)
                      && nearlyEqual(path->f, 0.0),
                      "mode " + to_string(mode) + ": start == goal gives one node with f = g = h = 0");
                deletePath(path);
            }
        }

        void testDepthPenalty()
        {
            int cases[4][3] = {{0, 0, 10}, {0, 50, 15}, {0, 100, 20}, {80, 30, 15}};

            for (int c = 0; c < 4; c++)
            {
                double matrix[100][100];
                int coords[100][3];
                clearMatrix(matrix);
                clearCoords(coords);
                addEdge(matrix, 0, 1, 10);
                setCoord(coords, 0, 0, 0, cases[c][0]);
                setCoord(coords, 1, 0, 0, cases[c][1]);

                PathNode* path = findAUVPath(matrix, coords, 0, 1, 1);
                check(nearlyEqual(finalCost(path), cases[c][2]) && validatePath(matrix, coords, path, 0, 1, 1),
                      "z " + to_string(cases[c][0]) + " -> " + to_string(cases[c][1])
                      + " on weight 10 costs " + to_string(cases[c][2]));
                deletePath(path);
            }
        }

        void testDepthChangesRoute()
        {
            double matrix[100][100];
            int coords[100][3];
            clearMatrix(matrix);
            clearCoords(coords);
            setCoord(coords, 0, 0, 0, 0);
            setCoord(coords, 1, 1, 0, 100);
            setCoord(coords, 2, 1, 0, 0);
            setCoord(coords, 3, 2, 0, 0);
            addEdge(matrix, 0, 1, 10);
            addEdge(matrix, 1, 3, 10);
            addEdge(matrix, 0, 2, 15);
            addEdge(matrix, 2, 3, 15);

            for (int mode = 1; mode <= 3; mode++)
            {
                PathNode* path = findAUVPath(matrix, coords, 0, 3, mode);
                check(describe(path) == "0 -> 2 -> 3" && nearlyEqual(finalCost(path), 30.0)
                      && validatePath(matrix, coords, path, 0, 3, mode),
                      "mode " + to_string(mode) + ": takes the flat route (30) over the deep route (40)");
                deletePath(path);
            }
        }

        void testUnreachable()
        {
            double matrix[100][100];
            int coords[100][3];
            clearMatrix(matrix);
            clearCoords(coords);
            addEdge(matrix, 0, 1, 2);
            addEdge(matrix, 2, 3, 2);

            for (int mode = 1; mode <= 3; mode++)
            {
                PathNode* path = findAUVPath(matrix, coords, 0, 3, mode);
                check(path == nullptr, "mode " + to_string(mode) + ": other component returns nullptr");
                deletePath(path);
            }
        }

        void testHeuristicComparison()
        {
            double matrix[100][100];
            int coords[100][3];
            clearMatrix(matrix);
            clearCoords(coords);
            setCoord(coords, 0, 0, 0, 0);
            setCoord(coords, 1, 10, 4, 4);
            setCoord(coords, 2, 5, 0, 0);
            setCoord(coords, 3, 10, 0, 0);
            addEdge(matrix, 0, 1, 2);
            addEdge(matrix, 1, 3, 6);
            addEdge(matrix, 0, 2, 4.5);
            addEdge(matrix, 2, 3, 4.5);

            double optimal = referenceCost(matrix, coords, 0, 3);
            check(nearlyEqual(optimal, 8.32), "Dijkstra reference finds the optimal cost 8.32");

            const char* names[] = {"", "Manhattan", "Euclidean", "Chebyshev"};
            double expected[] = {0, 9.0, 8.32, 8.32};
            for (int mode = 1; mode <= 3; mode++)
            {
                PathNode* path = findAUVPath(matrix, coords, 0, 3, mode);
                cout << "    " << names[mode] << ": " << describe(path)
                     << " | cost " << finalCost(path) << " | optimal " << optimal << "\n";
                check(nearlyEqual(finalCost(path), expected[mode])
                      && validatePath(matrix, coords, path, 0, 3, mode),
                      string(names[mode]) + " returns cost " + to_string(expected[mode]).substr(0, 4));
                deletePath(path);
            }
        }

        void testAllPairs(int numNodes)
        {
            double matrix[100][100];
            int coords[100][3];
            generateGeometricGraph(matrix, coords, numNodes);

            for (int mode = 1; mode <= 3; mode++)
            {
                int mismatches = 0;
                int checkedPairs = 0;
                for (int s = 0; s < numNodes; s++)
                {
                    for (int t = 0; t < numNodes; t++)
                    {
                        double expected = referenceCost(matrix, coords, s, t);
                        PathNode* path = findAUVPath(matrix, coords, s, t, mode);
                        ++checkedPairs;

                        if (expected == INFINITY)
                        {
                            if (path != nullptr) ++mismatches;
                        }
                        else if (path == nullptr || !validatePath(matrix, coords, path, s, t, mode)
                                 || !nearlyEqual(finalCost(path), expected))
                        {
                            ++mismatches;
                            cout << "    mismatch mode " << mode << ": " << s << " -> " << t << "\n";
                        }
                        deletePath(path);
                    }
                }
                check(mismatches == 0, "mode " + to_string(mode) + ": all " + to_string(checkedPairs)
                                       + " pairs match Dijkstra on " + to_string(numNodes) + " geometric nodes");
            }
        }

        void run()
        {
            testSamePoint();
            testDepthPenalty();
            testDepthChangesRoute();
            testUnreachable();
            testHeuristicComparison();
            testAllPairs(8);
            testAllPairs(30);
        }
    }

    namespace Task3
    {
        void run() { ++skipped; cout << "  not implemented yet\n"; }
    }

    namespace Task4
    {
        void run() { ++skipped; cout << "  not implemented yet\n"; }
    }

    namespace Task5
    {
        void run() { ++skipped; cout << "  not implemented yet\n"; }
    }

    struct Suite
    {
        const char* key;
        const char* title;
        void (*run)();
    };

    const Suite suites[] = {
        {"0", "Common: PathNode linked list", Common::run},
        {"1", "Task 1: Mesh network routing", Task1::run},
        {"2", "Task 2: AUV navigation", Task2::run},
        {"3", "Task 3: Probabilistic pathfinding", Task3::run},
        {"4", "Task 4: Game navigation", Task4::run},
        {"5", "Task 5: PCB routing", Task5::run},
    };
}

int main(int argc, char* argv[])
{
    string selected = argc > 1 ? argv[1] : "all";

    for (const Suite& suite : suites)
    {
        if (selected != "all" && selected != suite.key) continue;
        cout << "\n=== " << suite.title << " ===\n";
        suite.run();
    }

    cout << "\nPassed: " << passed << "  Failed: " << failed
         << "  Not implemented: " << skipped << "\n";
    return failed == 0 ? 0 : 1;
}