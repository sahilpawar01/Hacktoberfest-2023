//Let's implement a project that utilizes Dijkstra's algorithm for finding the shortest path in a weighted graph. We'll create a simple navigation system.



#include <iostream>
#include <vector>
#include <queue>
#include <limits.h>

class Graph {
private:
    int V;
    std::vector<std::vector<std::pair<int, int>>> adj;

public:
    Graph(int V) : V(V) {
        adj.resize(V);
    }

    void addEdge(int u, int v, int w) {
        adj[u].push_back(std::make_pair(v, w));
        adj[v].push_back(std::make_pair(u, w));
    }

    void dijkstra(int src) {
        std::vector<int> dist(V, INT_MAX);
        dist[src] = 0;
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
        pq.push(std::make_pair(0, src));

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            for (auto& neighbor : adj[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                    pq.push(std::make_pair(dist[v], v));
                }
            }
        }

        std::cout << "Shortest distances from node " << src << ":\n";
        for (int i = 0; i < V; ++i)
            std::cout << "Node " << i << " - Distance: " << dist[i] << std::endl;
    }
};

int main() {
    Graph g(5);
    g.addEdge(0, 1, 4);
    g.addEdge(0, 2, 1);
    g.addEdge(2, 1, 2);
    g.addEdge(2, 3, 4);
    g.addEdge(1, 3, 1);
    g.addEdge(3, 4, 5);

    g.dijkstra(0);

    return 0;
}
