#ifndef PATH_NODE_H
#define PATH_NODE_H

#include <string>

// Shared linked-list node used to represent the final path for all five tasks.
struct PathNode {
    std::string name;
    double f;
    double g;
    double h;
    PathNode* next;
};

// Creates one path node. The returned node is not linked to another node.
PathNode* createPathNode(const std::string& name, double f, double g, double h);

// Releases every node in a path linked list.
void deletePath(PathNode* head);

#endif
