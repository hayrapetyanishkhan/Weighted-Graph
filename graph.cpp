#include "graph.hpp"

Graph::Graph(const std::size_t n)
{
    adjList.resize(n);
}

void Graph::addEdge(const int u, const int v, const int w)
{
    if (u >= adjList.size() || u < 0 || v >= adjList.size() || v < 0) {
        throw std::out_of_range("Passed invalid vertices");
    }
    if (std::find(adjList[u].begin(), adjList[u].end(), 
        std::make_pair(v, w)) == adjList[u].end()) {
        adjList[u].push_back(std::make_pair(v, w));
    }
}

void Graph::traverse(std::function<void(const int, std::vector<bool>&)> Helper) const
{   
    std::vector<bool> visited(adjList.size(), false);

    for (int v{} ; v < visited.size() ; ++v) {
        if (!visited[v]) {
            Helper(v, visited);
        }
    }
}

void Graph::dfs() const
{
    traverse([this](const int u, std::vector<bool>& visited) 
    { 
        dfsHelper(u, visited); 
    });
}

void Graph::dfsHelper(const int u, std::vector<bool>& visited) const
{ 
    std::cout << u << " ";
    visited[u] = true;

    for (const auto& [v, _] : adjList[u]) {
        if (!visited[v]) {
            dfsHelper(v, visited);
        }
    }
}

void Graph::dfsIterative() const
{
    traverse([this](const int u, std::vector<bool>& visited) 
    { 
        dfsIterativeHelper(u, visited); 
    });
}

void Graph::dfsIterativeHelper(const int u, std::vector<bool>& visited) const
{
    std::stack<int> stack;
    stack.push(u);

    visited[u] = true;

    while (!stack.empty()) {
        int p = stack.top();
        stack.pop();

        std::cout << p << " ";

        for (const auto& [v, _] : adjList[p]) {
            if (!visited[v]) {
                visited[v] = true;
                stack.push(v);
            }
        }
    }
}

void Graph::bfs() const
{
    traverse([this](const int u, std::vector<bool>& visited) 
    { 
        bfsHelper(u, visited);
    });
}

void Graph::bfsHelper(const int u, std::vector<bool>& visited) const
{
    std::queue<int> q;
    q.push(u);

    visited[u] = true;

    while (!q.empty()) {
        int size = q.size();

        while (size--) {
            int p = q.front();
            q.pop();

            std::cout << p << " ";

            for (const auto& [v, _] : adjList[p]) {
                if (!visited[v]) {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }
        std::cout << std::endl;
    }
}

int Graph::numberOfNodesAtTheLevel(std::size_t level, const int start) const
{
    if (start < 0 || start >= adjList.size()) {
        throw std::out_of_range("Invalid start vertex");
    }
    std::vector<bool> visited(adjList.size(), false);
    visited[start] = true;

    std::queue<int> q;
    q.push(start);

    while (!q.empty()) {
        int size = q.size();
        if (level == 0) {
            return size;
        }
        while (size--) {
            int u = q.front();
            q.pop();
            for (const auto& [v, _] : adjList[u]) {
                if (!visited[v]) {
                    q.push(v);
                    visited[v] = true;
                }
            }
        }
        --level;
    }
    return 0;
}

Graph Graph::transpose() const
{
    Graph transposed(adjList.size());
    
    for (int u = 0; u < adjList.size(); ++u) {
        for (const auto& [v, w] : adjList[u]) {
            transposed.adjList[v].push_back(std::make_pair(u, w));
        }
    }

    return transposed;
}

bool Graph::canReachFromSrcToDst(const int src, const int dst) const
{
    if (src < 0 || src >= adjList.size() || dst < 0 || dst >= adjList.size()) {
        throw std::out_of_range("Invalid vertices");
    }
    std::vector<bool> visited(adjList.size(), false);
    return srcToDstHelper(src, dst, visited);
}

bool Graph::srcToDstHelper(const int src, const int dst, 
                           std::vector<bool>& visited) const
{
    if (dst == src) {
        return true;
    }
    visited[src] = true;
    for (const auto& [v, _] : adjList[src]) {
        if (!visited[v]) {
            if (srcToDstHelper(v, dst, visited)) {
                return true;
            }
        }
    }
    return false;
}

std::vector<std::vector<int>> Graph::getAllPathsFromSrcToDst(const int src, 
                                                             const int dst) const
{
    if (src < 0 || src >= adjList.size() || dst < 0 || dst >= adjList.size()) {
        throw std::out_of_range("Invalid vertices");
    }
    std::vector<std::vector<int>> result;
    std::vector<int> path;
    std::vector<bool> visited(adjList.size(), false);
    allPathsHelper(src, dst, path, result, visited);
    return result;
}

void Graph::allPathsHelper(const int src, const int dst, 
                           std::vector<int>& path, 
                           std::vector<std::vector<int>>& result, 
                           std::vector<bool>& visited) const
{
    path.push_back(src);
    if (src == dst) {
        result.push_back(path);
        path.pop_back();
        return;
    }
    visited[src] = true;
    for (const auto& [v, _] : adjList[src]) {
        if (!visited[v]) {
            allPathsHelper(v, dst, path, result, visited);
        }
    }
    path.pop_back();
    visited[src] = false;
}

bool Graph::hasCycle() const
{
    std::vector<bool> visited(adjList.size(), false);
    std::vector<bool> onStack(adjList.size(), false);
    for (int v{} ; v < adjList.size() ; ++v) {
        if (!visited[v]) {
            if (cycleHelper(v, visited, onStack)) {
                return true;
            }
        }
    }
    return false;
}

bool Graph::cycleHelper(const int u, std::vector<bool>& visited, 
                        std::vector<bool>& onStack) const
{
    if (onStack[u]) {
        return true;
    }
    onStack[u] = true;
    visited[u] = true;

    for (const auto& [v, _] : adjList[u]) {
        if (!visited[v]) {
            if (cycleHelper(v, visited, onStack)) {
                return true;
            }
        }
    }
    onStack[u] = false;
    return false;
}

void Graph::Kahn() const
{
    std::vector<int> inDegree(adjList.size(), 0);
    for (int u{} ; u < adjList.size() ; ++u) {
        for (const auto& [v, _] : adjList[u]) {
            ++inDegree[v];
        }
    }
    std::queue<int> q;
    for (int u{} ; u < adjList.size() ; ++u) {
        if (!inDegree[u]) {
            q.push(u);
        }
    }
    std::vector<int> result;
    while (!q.empty()) {
        int v = q.front();
        result.push_back(v);
        q.pop();
        for (const auto& [p, _] : adjList[v]) {
            --inDegree[p];
            if (!inDegree[p]) {
                q.push(p);
            }
        }
    }
    if (result.size() != adjList.size()) {
        std::cout << "The Graph has a loop" << std::endl;
        return;
    }
    for (int x : result) {
        std::cout << x << " ";
    }
}

std::vector<int> Graph::topSort() const
{
    std::vector<bool> visited(adjList.size(), false);
    std::vector<int> result;
    for (int u{} ; u < adjList.size() ; ++u) {
        if (!visited[u]) {
            vecDfs(u, result, visited);
        }
    }
    std::reverse(result.begin(), result.end());
    return result;
}

void Graph::vecDfs(const int u, std::vector<int>& result, 
                          std::vector<bool>& visited) const
{
    visited[u] = true;
    for (const auto& [v, _] : adjList[u]) {
        if (!visited[v]) {
            vecDfs(v, result, visited);
        }
    }
    result.push_back(u);
}

void Graph::dfsKosaraju(const int u, 
                        std::vector<bool>& visited, 
                        std::stack<int>& st) const
{
    visited[u] = true;
    for (const auto& [v, _] : adjList[u]) {
        if (!visited[v]) {
            dfsKosaraju(v, visited, st);
        }
    }
    st.push(u);
}

std::vector<std::vector<int>> Graph::kosaraju() const
{
    std::vector<std::vector<int>> sccs;
    std::vector<bool> visited(adjList.size(), false);
    std::stack<int> st;

    for (int u{}; u < visited.size(); ++u) {
        if (!visited[u]) {
            dfsKosaraju(u, visited, st);
        }
    }

    Graph transposed{transpose()};
    std::fill(visited.begin(), visited.end(), false);

    while (!st.empty()) {
        int u = st.top();
        st.pop();
        if (!visited[u]) {
            std::vector<int> vec;
            transposed.vecDfs(u, vec, visited);
            sccs.push_back(vec);
        }
    }
    return sccs;
}

std::vector<std::vector<int>> Graph::tarjan() const
{
    int id{};
    std::vector<std::vector<int>> sccs;
    std::vector<bool> onStack(adjList.size(), false);
    std::vector<int> ids(adjList.size(), -1);
    std::stack<int> st;
    std::vector<int> ll(adjList.size(), -1);

    for (int u{}; u < ids.size(); ++u) {
        if (ids[u] == -1) {
            tarjanHelper(u, id, ids, ll, st, onStack, sccs);
        }
    }
    return sccs;
}

void Graph::tarjanHelper(const int u, int& id, std::vector<int>& ids, 
                         std::vector<int>& ll, std::stack<int>& st, 
                         std::vector<bool>& onStack, 
                         std::vector<std::vector<int>>& sccs) const
{
    st.push(u);
    ids[u] = ll[u] = id++;
    onStack[u] = true;
    
    for (const auto& [v, _] : adjList[u]) {
        if (ids[v] == -1) {
            tarjanHelper(v, id, ids, ll, st, onStack, sccs);
        }
        if (onStack[v]) {
            ll[u] = std::min(ll[u], ll[v]);
        }
    }
    if (ll[u] == ids[u]) {
        std::vector<int> vec;
        while (1) {
            int p = st.top();
            st.pop();
            vec.push_back(p);
            onStack[p] = false;
            if (p == u) break;
        }
        sccs.push_back(vec);
    }
}

std::vector<std::pair<int, int>> Graph::dijkstra(const int start) const
{
    if (start < 0 || start >= adjList.size()) {
        throw std::out_of_range("Invalid start vertex");
    }

    std::vector<std::pair<int, int>> distance(adjList.size(), {std::numeric_limits<int>::max(), -1});
    distance[start].first = 0;

    std::priority_queue<std::pair<int, int>,
    std::vector<std::pair<int, int>>, std::greater<std::pair<int,int>>> pq;

    pq.push({0, start});
    
    while (!pq.empty()) {
        
        const auto [dist, v] = pq.top();
        pq.pop();

        if (dist > distance[v].first) continue;

        for (const auto& [p, weight] : adjList[v]) {
            if (distance[p].first > dist + weight) {
                distance[p].first = dist + weight;
                distance[p].second = v;
                pq.push({dist + weight, p});
            }
        }
    }

    return distance;
}