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

        void run()
        {
            testSamePoint();
            testDirectEdge();
            testDegreePenalty();
            testUnreachable();
            testGeneratedSample();
            testAllPairs(6);
            testAllPairs(15);
            testAllPairs(40);
        }
    }

    namespace Task2
    {
        void run() { ++skipped; cout << "  not implemented yet\n"; }
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