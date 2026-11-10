

#include "PathNode.h"
#include "Algo.h"

// Local testing helper based on the printPath example in Section 2.3.
// The LMS provides its own printPath function: do not submit this helper
// as part of the LMS auto-tester code.

void generateWeightedMatrix(double matrix[100][100], int numNodes)
{
    // Initialize everything to zero
    for (int i = 0; i < numNodes; i++)
    {
        for (int j = 0; j < numNodes; j++)
        {
            matrix[i][j] = 0;
        }
    }

    // Generate connections using a deterministic pattern
    for (int i = 0; i < numNodes; i++)
    {
        for (int j = i + 1; j < numNodes; j++)
        {
            // Create a connection when the condition is satisfied
            if ((i + j) % 3 != 0)
            {
                double weight = ((i + 1) * (j + 2)) % 20 + 1;

                matrix[i][j] = weight;
                matrix[j][i] = weight;
            }
        }
    }
}

void printMatrix(double matrix[100][100], int numNodes)
{
    for (int i = 0; i < numNodes; i++)
    {
        for (int j = 0; j < numNodes; j++)
        {
            cout << matrix[i][j] << "\t";
        }

        cout << '\n';
    }
}


void printPath(PathNode* head) {
    cout << "Solution Path:\n";

    while (head != nullptr) {
        cout << "Node: " << head->name
                  << " | f: " << head->f
                  << " | g: " << head->g
                  << " | h: " << head->h << "\n";

        head = head->next;
    }
}

int main() {
    // Temporary example to verify PathNode creation, linking, and printing.
    // Replace this demonstration with real task tests as the algorithms are built.
    PathNode* start = createPathNode("Start", 4.0, 0.0, 4.0);
    PathNode* middle = createPathNode("Middle", 3.0, 1.0, 2.0);
    PathNode* goal = createPathNode("Goal", 4.0, 4.0, 0.0);

    start->next = middle;
    middle->next = goal;

    printPath(start);
    deletePath(start);


    const int NUM_NODES = 6;
    double matrix[100][100] = {};

    generateWeightedMatrix(matrix, NUM_NODES);

    cout << "Generated weighted matrix:\n";
    printMatrix(matrix, NUM_NODES);

    findMeshPath(matrix, 0, 1);


    return 0;
}
