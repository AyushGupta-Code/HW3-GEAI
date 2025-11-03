#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <chrono>
using namespace std;

struct Edge { int to; double w; };

// --- Read edges from CSV ---
unordered_map<int, vector<Edge>> loadEdges(const string& filename) {
    unordered_map<int, vector<Edge>> graph;
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "❌ Failed to open " << filename << endl;
        exit(1);
    }

    string line;
    getline(fin, line); // skip header
    while (getline(fin, line)) {
        stringstream ss(line);
        string su, sv, sw;
        getline(ss, su, ',');
        getline(ss, sv, ',');
        getline(ss, sw, ',');
        if (su.empty() || sv.empty() || sw.empty()) continue;

        int u = stoi(su), v = stoi(sv);
        double w = stod(sw);
        graph[u].push_back({v, w});
    }
    cerr << "✓ Loaded edges from " << filename << " (" << graph.size() << " nodes)\n";
    return graph;
}

// --- Read heuristics from CSV ---
unordered_map<int, double> loadHeuristics(const string& filename, bool admissible = true) {
    unordered_map<int, double> H;
    ifstream fin(filename);
    if (!fin.is_open()) {
        cerr << "❌ Failed to open " << filename << endl;
        exit(1);
    }

    string line;
    getline(fin, line); // header
    while (getline(fin, line)) {
        stringstream ss(line);
        string sid, sh1, sh2;
        getline(ss, sid, ',');
        getline(ss, sh1, ',');
        getline(ss, sh2, ',');
        if (sid.empty() || sh1.empty()) continue;

        int id = stoi(sid);
        double val = admissible ? stod(sh1) : stod(sh2);
        H[id] = val;
    }
    cerr << "✓ Loaded heuristics from " << filename << " (" << H.size() << " nodes)\n";
    return H;
}

// --- A* Search ---
vector<int> astar(const unordered_map<int, vector<Edge>>& graph,
                  int start, int goal,
                  const unordered_map<int, double>& H) {
    using P = pair<double, int>; // (f, node)
    unordered_map<int, double> g;
    unordered_map<int, double> f;
    unordered_map<int, int> parent;

    for (auto& [u, _] : graph) {
        g[u] = numeric_limits<double>::infinity();
        f[u] = numeric_limits<double>::infinity();
    }

    g[start] = 0;
    f[start] = H.at(start);

    priority_queue<P, vector<P>, greater<P>> open;
    open.push({f[start], start});

    unordered_set<int> closed;
    int expansions = 0;

    while (!open.empty()) {
        auto [fcur, u] = open.top(); open.pop();
        if (closed.count(u)) continue;
        closed.insert(u);
        expansions++;

        if (u == goal) {
            cerr << "🎯 Goal reached after expanding " << expansions << " nodes.\n";
            vector<int> path;
            for (int v = goal; v != start; v = parent[v]) path.push_back(v);
            path.push_back(start);
            reverse(path.begin(), path.end());
            cerr << "✅ Path cost: " << g[goal] << endl;
            return path;
        }

        if (!graph.count(u)) continue;
        for (auto [v, w] : graph.at(u)) {
            double tentative = g[u] + w;
            if (tentative < g[v]) {
                g[v] = tentative;
                f[v] = tentative + H.at(v);
                parent[v] = u;
                open.push({f[v], v});
            }
        }
    }

    cerr << "❌ Goal not reachable.\n";
    return {};
}

int main() {
    string edgesFile = "large_graph_edges.csv";
    string heurFile  = "large_graph_heuristics.csv";

    auto graph = loadEdges(edgesFile);
    auto H = loadHeuristics(heurFile, true); // true = admissible heuristic

    int start = 0;  // you can modify these IDs
    int goal  = 100; // choose a valid node ID within your range

    auto t1 = chrono::high_resolution_clock::now();
    vector<int> path = astar(graph, start, goal, H);
    auto t2 = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

    cout << "\n--- A* Results ---\n";
    cout << "Start: " << start << "  Goal: " << goal << endl;
    cout << "Runtime: " << duration << " ms\n";
    cout << "Path length: " << path.size() << endl;
    cout << "Path: ";
    for (int v : path) cout << v << " ";
    cout << "\n";
}
