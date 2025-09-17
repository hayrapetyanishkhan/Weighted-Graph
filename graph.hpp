#ifndef WEIGHTED_GRAPH
#define WEIGHTED_GRAPH

#include <iostream>
#include <vector>
#include <functional>
#include <ranges>
#include <algorithm>
#include <queue>

class Graph{
    private:
        std::vector<std::vector<std::pair<int, int>>> adjList;
    public:
        Graph(const std::size_t n);
    
        void addEdge(const int u, const int v, const int w);
    
        void dfs() const;
        void dfsIterative() const;
    
        void bfs() const;
    
        int numberOfNodesAtTheLevel(std::size_t level, const int start = 0) const;
    
        Graph transpose() const;
    
        bool canReachFromSrcToDst(const int src, const int dst) const;
    
        std::vector<std::vector<int>> getAllPathsFromSrcToDst(const int src, 
                                                              const int dst) const;
    
        bool hasCycle() const;
    
        void Kahn() const;
    
        std::vector<int> topSort() const;
    
        std::vector<std::vector<int>> kosaraju() const;
    
        std::vector<std::vector<int>> tarjan() const;

        std::vector<std::pair<int, int>> dijkstra(const int start) const;

    private:
        void traverse(std::function<void(const int, std::vector<bool>&)> Helper) const;
            
        void dfsHelper(const int u, std::vector<bool>& visited) const;
        void dfsIterativeHelper(const int u, std::vector<bool>& visited) const;
    
        void bfsHelper(const int u, std::vector<bool>& visited) const;
    
        bool srcToDstHelper(const int src, const int dst, 
                            std::vector<bool>& visited) const;
    
        void allPathsHelper(const int src, const int dst, 
                            std::vector<int>& path, 
                            std::vector<std::vector<int>>& result, 
                            std::vector<bool>& visited) const;
    
        bool cycleHelper(const int u, std::vector<bool>& visited, 
                         std::vector<bool>& onStack) const;
    
        void vecDfs(const int u, std::vector<int>& result, 
                           std::vector<bool>& visited) const;
    
        void tarjanHelper(const int u, int& id, 
                          std::vector<int>& ids, 
                          std::vector<int>& ll,
                          std::stack<int>& st,
                          std::vector<bool>& onStack, 
                          std::vector<std::vector<int>>& sccs) const;
        
        void dfsKosaraju(const int u, 
                         std::vector<bool>& visited, 
                         std::stack<int>& st) const;
    };


#endif
