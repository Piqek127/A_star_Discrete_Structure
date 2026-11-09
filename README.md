# A* Discrete Structure Assignment

HCMUT — CO1007 Discrete Structure, Semester 1, 2026–2027.

This repository contains the C++ implementation of the five A* pathfinding tasks in the assignment.

## Required source files

| File | Responsibility |
|---|---|
| `PathNode.h` | Shared `PathNode` linked-list structure and helper declarations |
| `PathNode.cpp` | Path-node creation and linked-list cleanup |
| `Algo.h` | Required function declarations for Tasks 1–5 |
| `Algo.cpp` | A* implementations for Tasks 1–5 |
| `main.cpp` | Local demonstration and testing |

## Required task functions

- `findMeshPath`
- `findAUVPath`
- `findProbabilisticPath`
- `findGamePath`
- `findPCBPath`

The function signatures in `Algo.h` follow Section 2.2 of the assignment. The shared linked-list node follows Section 2.3.

## Current status

- [x] Required file structure created
- [x] Shared `PathNode` structure defined
- [x] Five required function declarations added
- [x] Local linked-list demonstration added
- [ ] Task 1 — Mesh network routing
- [ ] Task 2 — AUV navigation
- [ ] Task 3 — Probabilistic pathfinding
- [ ] Task 4 — Game navigation
- [ ] Task 5 — PCB routing

## Local compilation

With a C++ compiler that supports C++11 or later:

```bash
g++ -std=c++11 -Wall -Wextra -pedantic PathNode.cpp Algo.cpp main.cpp -o astar_demo
./astar_demo
```

The current task functions are placeholders and return `nullptr`. The sample in `main.cpp` only tests the shared linked-list structure.

## Important LMS note

The LMS provides `printPath(PathNode* head)` in its test system. The copy in `main.cpp` is only for local testing and should not be included in an LMS code-runner submission.
