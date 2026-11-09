#include <iostream>

#include "PathNode.h"
#include "Algo.h"

// Local testing helper based on the printPath example in Section 2.3.
// The LMS provides its own printPath function: do not submit this helper
// as part of the LMS auto-tester code.
void printPath(PathNode* head) {
    std::cout << "Solution Path:\n";

    while (head != nullptr) {
        std::cout << "Node: " << head->name
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

    return 0;
}
