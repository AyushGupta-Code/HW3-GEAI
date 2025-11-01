#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

// ====================== Structures ======================
struct Edge { int to; float w; bool directed; };
struct Node { int id; string name; float x, y; };

struct Graph {
    unordered_map<int, vector<Edge>> adj;
    unordered_map<string, int> nameToId;
    vector<Node> nodes;

    static string trim(const string& s) {
        size_t b = 0, e = s.size();
        while (b < e && isspace((unsigned char)s[b])) ++b;
        while (e > b && isspace((unsigned char)s[e - 1])) --e;
        return s.substr(b, e - b);
    }

    void addNode(int id, const string& name, float x, float y) {
        if ((int)nodes.size() <= id) nodes.resize(id + 1);
        nodes[id] = {id, name, x, y};
        nameToId[name] = id;
    }

    void addEdge(int u, int v, float w, bool directed) {
        adj[u].push_back({v, w, directed});
        if (!directed) adj[v].push_back({u, w, directed});
    }
};

// ====================== CSV Loaders ======================
void load_nodes(Graph& g, const string& filename) {
    ifstream f(filename);
    if (!f) { cerr << "Error: cannot open " << filename << endl; exit(1); }

    string line;
    getline(f, line);
    while (getline(f, line)) {
        if (Graph::trim(line).empty()) continue;
        stringstream ss(line);
        string idStr, name, xStr, yStr;
        getline(ss, idStr, ',');
        getline(ss, name, ',');
        getline(ss, xStr, ',');
        getline(ss, yStr, ',');
        if (idStr.empty() || name.empty()) continue;
        int id = stoi(idStr);
        float x = xStr.empty() ? 0 : stof(xStr);
        float y = yStr.empty() ? 0 : stof(yStr);
        g.addNode(id, Graph::trim(name), x, y);
    }
}

void load_edges(Graph& g, const string& filename) {
    ifstream f(filename);
    if (!f) { cerr << "Error: cannot open " << filename << endl; exit(1); }

    string line;
    getline(f, line);
    while (getline(f, line)) {
        if (Graph::trim(line).empty()) continue;
        stringstream ss(line);
        string fromStr, toStr, wStr, dStr;
        getline(ss, fromStr, ',');
        getline(ss, toStr, ',');
        getline(ss, wStr, ',');
        getline(ss, dStr, ',');
        if (fromStr.empty() || toStr.empty() || wStr.empty()) continue;
        int u = stoi(fromStr);
        int v = stoi(toStr);
        float w = stof(wStr);
        bool directed = (!dStr.empty() && stoi(dStr) != 0);
        g.addEdge(u, v, w, directed);
    }
}

// ====================== Dijkstra ======================
struct DijkstraStats {
    size_t expansions = 0;
    size_t maxFringe  = 0;
    double ms         = 0.0;
    float  pathCost   = INFINITY;
};

vector<int> dijkstra(const Graph& g, const string& startName, const string& goalName, DijkstraStats& stats) {
    auto findId = [&](const string& s)->int {
        auto it = g.nameToId.find(s);
        return (it == g.nameToId.end() ? -1 : it->second);
    };
    int start = findId(startName);
    int goal  = findId(goalName);
    if (start < 0 || goal < 0) {
        cerr << "Unknown start/goal: " << startName << " -> " << goalName << endl;
        return {};
    }

    const int N = g.nodes.size();
    vector<float> dist(N, INFINITY);
    vector<int> parent(N, -1);
    vector<char> closed(N, 0);

    using PQItem = pair<float,int>;
    priority_queue<PQItem, vector<PQItem>, greater<PQItem>> open;
    dist[start] = 0.0f;
    open.push({0.0f, start});

    auto t0 = chrono::high_resolution_clock::now();

    while (!open.empty()) {
        stats.maxFringe = max(stats.maxFringe, open.size());
        int u = open.top().second; open.pop();
        if (closed[u]) continue;
        closed[u] = 1;
        stats.expansions++;

        if (u == goal) break;

        auto it = g.adj.find(u);
        if (it == g.adj.end()) continue;

        for (const auto& e : it->second) {
            if (closed[e.to]) continue;
            float alt = dist[u] + e.w;
            if (alt < dist[e.to]) {
                dist[e.to] = alt;
                parent[e.to] = u;
                open.push({alt, e.to});
            }
        }
    }

    auto t1 = chrono::high_resolution_clock::now();
    stats.ms = chrono::duration<double, milli>(t1 - t0).count();
    stats.pathCost = dist[goal];

    if (parent[goal] == -1 && start != goal) return {};

    vector<int> path;
    for (int v = goal; v != -1; v = parent[v]) {
        path.push_back(v);
        if (v == start) break;
    }
    reverse(path.begin(), path.end());
    return path;
}

// ====================== Utility ======================
float path_cost(const Graph& g, const vector<int>& path) {
    float cost = 0.0f;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        int u = path[i], v = path[i + 1];
        float edgeCost = INFINITY;
        auto it = g.adj.find(u);
        if (it != g.adj.end()) {
            for (const auto& e : it->second)
                if (e.to == v) { edgeCost = e.w; break; }
        }
        if (edgeCost == INFINITY) return INFINITY;
        cost += edgeCost;
    }
    return cost;
}

// ====================== MAIN ======================
int main() {
    Graph g;
    load_nodes(g, "nodes.csv");
    load_edges(g, "edges.csv");

    const string startName = "Dan Allen Deck";
    const string goalName  = "Bell Tower";

    DijkstraStats stats;
    auto path = dijkstra(g, startName, goalName, stats);

    cout << "Dijkstra from " << startName << " to " << goalName << ":\n";
    if (path.empty()) {
        cout << "No path found.\n";
        return 0;
    }

    for (size_t i = 0; i < path.size(); ++i) {
        cout << g.nodes[path[i]].name;
        if (i + 1 < path.size()) cout << " -> ";
    }

    float pc = path_cost(g, path);
    cout << "\nCost: " << fixed << setprecision(3) << pc
         << " | Runtime: " << fixed << setprecision(3) << stats.ms << " ms"
         << " | Expanded: " << stats.expansions
         << " | Max fringe: " << stats.maxFringe << "\n";
}
