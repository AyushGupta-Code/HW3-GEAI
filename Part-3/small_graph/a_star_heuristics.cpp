#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <string>
#include <iomanip>
#include <chrono>
#include <functional>
#include <algorithm>

using namespace std;

struct Node {
    int id;
    string name;
    double x, y;
    string cluster;
};

struct Edge {
    int from, to;
    double weight;
};

struct Graph {
    vector<Node> nodes;
    unordered_map<int, vector<pair<int,double>>> adj;
};

// ---------- Read CSV helpers ----------
vector<Node> readNodes(const string& filename) {
    vector<Node> nodes;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "❌ Failed to open " << filename << endl;
        exit(1);
    }

    string line;
    getline(file, line); // skip header

    while (getline(file, line)) {
        if (line.empty()) continue;
        stringstream ss(line);
        string id, name, x, y, cluster;

        getline(ss, id, ',');
        getline(ss, name, ',');
        getline(ss, x, ',');
        getline(ss, y, ',');
        getline(ss, cluster, ',');

        try {
            Node n;
            n.id = stoi(id);
            n.name = name;
            n.x = stod(x);
            n.y = stod(y);
            n.cluster = cluster.empty() ? "None" : cluster;
            nodes.push_back(n);
        } catch (const invalid_argument&) {
            cerr << "⚠️ Skipping invalid line: " << line << endl;
        }
    }
    return nodes;
}

vector<Edge> readEdges(const string& filename) {
    vector<Edge> edges;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "❌ Failed to open " << filename << endl;
        exit(1);
    }
    string line;
    getline(file, line); // skip header
    while (getline(file, line)) {
        if (line.empty()) continue;
        stringstream ss(line);
        string from, to, weight;
        getline(ss, from, ',');
        getline(ss, to, ',');
        getline(ss, weight, ',');

        try {
            edges.push_back({stoi(from), stoi(to), stod(weight)});
        } catch (const invalid_argument&) {
            cerr << "⚠️ Skipping invalid edge line: " << line << endl;
        }
    }
    return edges;
}

// ---------- Heuristics ----------
double euclideanHeuristic(const Node& a, const Node& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

double clusterHeuristic(const Node& a, const Node& b) {
    double base = euclideanHeuristic(a, b);
    if (a.cluster == b.cluster)
        return base;
    else
        return 1.5 * base; // overestimate for cross-cluster
}

// ---------- A* ----------
pair<vector<int>, double> aStar(
    const Graph& g, int start, int goal,
    function<double(const Node&, const Node&)> heuristic,
    int& expanded
) {
    unordered_map<int,double> gScore, fScore;
    unordered_map<int,int> cameFrom;
    for (auto& node : g.nodes) {
        gScore[node.id] = 1e18;
        fScore[node.id] = 1e18;
    }
    gScore[start] = 0;
    fScore[start] = heuristic(g.nodes[start], g.nodes[goal]);

    using P = pair<double,int>;
    priority_queue<P, vector<P>, greater<P>> open;
    open.push({fScore[start], start});

    expanded = 0;
    unordered_set<int> visited;

    while (!open.empty()) {
        auto [f, u] = open.top();
        open.pop();
        if (visited.count(u)) continue;
        visited.insert(u);
        expanded++;

        if (u == goal) break;

        for (auto [v, w] : g.adj.at(u)) {
            double tentative = gScore[u] + w;
            if (tentative < gScore[v]) {
                cameFrom[v] = u;
                gScore[v] = tentative;
                fScore[v] = tentative + heuristic(g.nodes[v], g.nodes[goal]);
                open.push({fScore[v], v});
            }
        }
    }

    vector<int> path;
    int cur = goal;
    while (cameFrom.find(cur) != cameFrom.end()) {
        path.push_back(cur);
        cur = cameFrom[cur];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return {path, gScore[goal]};
}

// ---------- MAIN ----------
int main() {
    string nodesFile = "nodes.csv";
    string edgesFile = "edges.csv";

    auto nodes = readNodes(nodesFile);
    auto edges = readEdges(edgesFile);

    Graph g;
    g.nodes = nodes;
    for (auto& e : edges)
        g.adj[e.from].push_back({e.to, e.weight});

    int start = 0, goal = g.nodes.size() - 1;
    int expanded1, expanded2;

    auto startTime = chrono::high_resolution_clock::now();
    auto [path1, cost1] = aStar(g, start, goal, euclideanHeuristic, expanded1);
    auto endTime = chrono::high_resolution_clock::now();
    double time1 = chrono::duration<double, milli>(endTime - startTime).count();

    startTime = chrono::high_resolution_clock::now();
    auto [path2, cost2] = aStar(g, start, goal, clusterHeuristic, expanded2);
    endTime = chrono::high_resolution_clock::now();
    double time2 = chrono::duration<double, milli>(endTime - startTime).count();

    cout << fixed << setprecision(3);
    cout << "\n=== Admissible (Euclidean) ===\n";
    cout << "Path: ";
    for (int p : path1) cout << g.nodes[p].name << " ";
    cout << "\nCost: " << cost1 << "\nNodes Expanded: " << expanded1
         << "\nRuntime: " << time1 << " ms\n";

    cout << "\n=== Inadmissible (Cluster) ===\n";
    cout << "Path: ";
    for (int p : path2) cout << g.nodes[p].name << " ";
    cout << "\nCost: " << cost2 << "\nNodes Expanded: " << expanded2
         << "\nRuntime: " << time2 << " ms\n";
}
