#include <bits/stdc++.h>
using namespace std;

struct Edge { int from, to, w; };

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    const string INPUT_FILE = "roadNet-CA.txt";  // decompressed SNAP file
    const string NODES_FILE = "nodes.csv";
    const string EDGES_FILE = "edges.csv";
    const string GRAPH_FILE = "graph.csv";
    const string HEUR_FILE  = "heuristics.csv";

    ifstream fin(INPUT_FILE);
    if (!fin.is_open()) {
        cerr << "❌ Cannot open " << INPUT_FILE << "\n";
        return 1;
    }

    cout << "📖 Reading edges from " << INPUT_FILE << " ...\n";
    vector<Edge> edges;
    int maxNode = -1;
    string line;
    while (getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        stringstream ss(line);
        int u, v;
        ss >> u >> v;
        if (!ss) continue;
        maxNode = max({maxNode, u, v});
        edges.push_back({u, v, rand() % 20 + 1});
    }
    fin.close();
    int N = maxNode + 1;
    cout << "✅ Loaded " << N << " nodes and " << edges.size() << " edges.\n";

    // Random positions for visualization
    vector<pair<float,float>> pos(N);
    mt19937_64 rng(12345);
    uniform_real_distribution<float> xdist(0, 5000), ydist(0, 5000);
    for (int i = 0; i < N; ++i)
        pos[i] = {xdist(rng), ydist(rng)};

    // Choose arbitrary goal for heuristic (node 0)
    int goal = 0;
    vector<int> heur(N);
    for (int i = 0; i < N; ++i) {
        float dx = pos[i].first - pos[goal].first;
        float dy = pos[i].second - pos[goal].second;
        heur[i] = static_cast<int>(sqrt(dx*dx + dy*dy) / 100.0f + 0.5f);
    }

    cout << "🧩 Writing CSV files ...\n";

    // nodes.csv
    ofstream nout(NODES_FILE);
    nout << "id,name,x,y\n";
    for (int i = 0; i < N; ++i)
        nout << i << ",Node_" << i << "," << pos[i].first << "," << pos[i].second << "\n";

    // edges.csv
    ofstream eout(EDGES_FILE);
    eout << "from,to,weight,directed\n";
    for (auto &e : edges)
        eout << e.from << "," << e.to << "," << e.w << ",1\n";

    // graph.csv
    ofstream gout(GRAPH_FILE);
    gout << "Source,Target,Weight\n";
    for (auto &e : edges)
        gout << "Node_" << e.from << ",Node_" << e.to << "," << e.w << "\n";

    // heuristics.csv
    ofstream hout(HEUR_FILE);
    hout << "Node,Heuristic_to_Node0\n";
    for (int i = 0; i < N; ++i)
        hout << "Node_" << i << "," << heur[i] << "\n";

    cout << "✅ Done.\n";
    cout << "   • " << NODES_FILE << "\n";
    cout << "   • " << EDGES_FILE << "\n";
    cout << "   • " << GRAPH_FILE << "\n";
    cout << "   • " << HEUR_FILE  << "\n";
}
