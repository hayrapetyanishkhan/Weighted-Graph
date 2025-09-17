#include "graph.hpp"

int main()
{
    Graph g(6);
    g.addEdge(0, 1, 1);
    g.addEdge(1, 2, 2);
    g.addEdge(1, 3, 3);
    g.addEdge(3, 0, 8);
    g.addEdge(4, 2, 6);
    g.addEdge(4, 5, 6);
    g.addEdge(5, 4, 6);

    // auto k = g.tarjan();
    // for (const auto& row : k) {
    //     for (int x : row) {
    //         std::cout << x;
    //     }
    //     std::cout << std::endl;
    // }

    auto res = g.dijkstra(0);
    for (int i{}; i < res.size(); ++i) {

        int distance = res[i].first;
        int parent = res[i].second;

        std::cout << "The path from " << i << " with total weight " << distance;
        std::cout << " : ";

        if (distance == std::numeric_limits<int>::max()) {
            std::cout << "Unreachable\n";
            continue;
        }

        std::vector<int> path;
        for (int at = i; at != -1; at = res[at].second) {
            path.push_back(at);
        }

        for (int i = path.size() - 1; i >= 0; --i) std::cout << path[i] << " ";

        std::cout << std::endl;
    }
    
    return 0;
}