#include <iostream>
#include <vector>
#include <queue>
#include <climits> // Required for INT_MAX

using namespace std;

class Router {
private:
    vector<int> dist;
    vector<int> parent;
    int n;
    vector<vector<pair<int, int>>> graph;

    void printPathRecursive(int targetNode) {
        if (parent[targetNode] != 0) {
            printPathRecursive(parent[targetNode]);
        }

        cout << targetNode << " ";
    }

    void dijkstra() {
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> q;

        dist[1] = 0;
        q.push({0, 1}); // {distance, vertex}

        while (!q.empty()) {
            int u_vertex = q.top().second;
            int u_dist = q.top().first;
            q.pop();
            
            // Optimization: If we've already found a better path, skip.
            if(u_dist > dist[u_vertex]) {
                continue;
            }

            for (auto& edge : graph[u_vertex]) {
                int v_vertex = edge.first;
                int weight = edge.second;

                if (dist[u_vertex] + weight < dist[v_vertex]) {
                    dist[v_vertex] = dist[u_vertex] + weight;
                    parent[v_vertex] = u_vertex;
                    q.push({dist[v_vertex], v_vertex});
                }
            }
        }
    }

public:
    Router(const vector<vector<pair<int, int>>>& input_graph, int num_nodes) : n(num_nodes) {
        dist.resize(n + 1, INT_MAX); // Initialize all distances to "infinity".
        parent.resize(n + 1, 0);     // Initialize all parents to 0.
        graph = input_graph;

        dijkstra(); 
    }

    void showPathTo(int destination) {
        if (destination > n || destination < 1 || dist[destination] == INT_MAX) {
            cout << "No path found from router 1 to " << destination << "." << endl;
        } else {
            cout << "Shortest distance to " << destination << " is: " << dist[destination] << endl;
            cout << "Path: ";
            printPathRecursive(destination);
            cout << endl;
        }
    }
};

int main() {
    int n, u, v, e, w;
    cout << "Input number of routers and links:\n";
    cin >> n >> e;

    // Use a vector of vectors, sized correctly.
    vector<vector<pair<int, int>>> graph(n + 1);

    cout << "Input " << e << " links (router1 router2 weight):\n";
    for (int i = 0; i < e; i++) {
        cin >> u >> v >> w;
        graph[u].push_back({v, w});
        graph[v].push_back({u, w});
    }

    cout << "CHAMA!\n";

    Router r1(graph, n);
    r1.showPathTo(n);

    return 0;
}