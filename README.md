# Graph Algorithms in C++

This project implements a **directed, weighted graph** in C++ with a wide set of algorithms.
It supports graph traversals, path finding, shortest path, strongly connected components, and cycle detection.

## ✨ Features

### Graph Representation

* Directed, weighted adjacency list

### Traversals

* Depth-First Search (DFS) — recursive & iterative
* Breadth-First Search (BFS)

### Path & Reachability

* Count nodes at a specific level from a start node
* Check if there is a path from source to destination
* Find all paths between two nodes

### Graph Properties

* Transpose of a graph
* Cycle detection (DFS-based)

### Topological Sorting

* Kahn’s Algorithm (BFS-based)
* DFS-based topological sort

### Strongly Connected Components (SCC)

* **Kosaraju’s Algorithm**
* **Tarjan’s Algorithm**

### Shortest Path

* **Dijkstra’s Algorithm** — returns shortest distances and parent paths

## 📂 Project Structure

```
graph.hpp      # Class definition
graph.cpp      # Implementation of Graph methods
main.cpp       # Example usage & testing
```

## 🚀 Build & Run

```bash
g++ -std=c++20 main.cpp graph.cpp -o graph
./graph
```

## 📌 Example Output (Dijkstra)

```
The path from 0 with total weight 0 : 0 
The path from 1 with total weight 1 : 0 1 
The path from 2 with total weight 3 : 0 1 2 
The path from 3 with total weight 4 : 0 1 3 
The path from 4 with total weight Unreachable
The path from 5 with total weight Unreachable
```

## 📖 Learning Outcomes

This project demonstrates:

* Graph theory algorithms (DFS, BFS, SCCs, shortest paths, topological sorting)
* Efficient use of **adjacency lists** for weighted graphs
* STL containers: `vector`, `queue`, `stack`, `priority_queue`
* Algorithmic techniques: recursion, iteration, backtracking, greedy shortest path
* Error handling with exceptions (`std::out_of_range`)

## 🚀 Future Improvements

* Add **Bellman-Ford** (handles negative weights)
* Add **Floyd-Warshall** (all-pairs shortest path)
* Implement **Prim’s & Kruskal’s** (Minimum Spanning Tree)
* Add graph visualization export (e.g., Graphviz `.dot` format)

## 📬 Author

**Ishkhan Hayrapetyan**
[GitHub Profile](https://github.com/hayrapetyanishkhan)
