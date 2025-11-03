#include <SFML/Graphics.hpp>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <iostream>

struct Node {
    int r, c;
    bool blocked = false;
    std::vector<Node*> nbrs;
};

constexpr int ROWS = 20;
constexpr int COLS = 30;
constexpr int CELL = 32;
constexpr float MAX_SPEED = 120.f;
constexpr float ARRIVE_RADIUS = 10.f;

// Euclidean heuristic for A*
float heuristic(Node* a, Node* b) {
    float dx = a->c - b->c, dy = a->r - b->r;
    return std::sqrt(dx * dx + dy * dy);
}

// --- A* search ---
std::vector<Node*> a_star(Node* start, Node* goal) {
    std::unordered_map<Node*, Node*> came;
    std::unordered_map<Node*, float> g, f;
    auto cmp = [&](Node* a, Node* b) { return f[a] > f[b]; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

    g[start] = 0;
    f[start] = heuristic(start, goal);
    open.push(start);

    while (!open.empty()) {
        Node* cur = open.top();
        open.pop();
        if (cur == goal) break;

        for (auto* n : cur->nbrs) {
            if (n->blocked) continue;
            float tentative = g[cur] + heuristic(cur, n);
            if (!g.count(n) || tentative < g[n]) {
                came[n] = cur;
                g[n] = tentative;
                f[n] = tentative + heuristic(n, goal);
                open.push(n);
            }
        }
    }

    std::vector<Node*> path;
    if (!came.count(goal)) return path;
    for (Node* cur = goal; cur; cur = came.count(cur) ? came[cur] : nullptr) {
        path.push_back(cur);
        if (cur == start) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

sf::Vector2f toWorld(Node* n) {
    return {n->c * CELL + CELL / 2.f, n->r * CELL + CELL / 2.f};
}

// --- Agent with seek/arrive ---
struct Agent {
    sf::CircleShape shape;
    std::vector<sf::Vector2f> path;
    sf::Vector2f vel{0.f, 0.f};
    size_t target = 0;

    Agent() {
        shape.setRadius(8.f);
        shape.setOrigin({8.f, 8.f});
        shape.setFillColor(sf::Color::Cyan);
    }

    void setPath(const std::vector<Node*>& nodes) {
        path.clear();
        for (auto* n : nodes) path.push_back(toWorld(n));
        target = 0;
    }

    void update(float dt) {
        if (target >= path.size()) return;
        sf::Vector2f pos = shape.getPosition();
        sf::Vector2f desired = path[target] - pos;
        float d = std::sqrt(desired.x * desired.x + desired.y * desired.y);
        if (d < ARRIVE_RADIUS) { target++; return; }
        desired /= d;
        float speed = (target == path.size() - 1 && d < 100) ? MAX_SPEED * (d / 100.f) : MAX_SPEED;
        sf::Vector2f steer = desired * speed - vel;
        vel += steer * dt;
        float mag = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        if (mag > MAX_SPEED) vel *= MAX_SPEED / mag;
        shape.move(vel * dt);
    }
};

int main() {
    // ✅ SFML 3.x fix: pass Vector2u to VideoMode
    sf::RenderWindow window(
        sf::VideoMode({static_cast<unsigned int>(COLS * CELL),
                       static_cast<unsigned int>(ROWS * CELL)}),
        "Dynamic A* Path Following (Corridor Layout)");
    window.setFramerateLimit(60);

    Node grid[ROWS][COLS];
    for (int r = 0; r < ROWS; ++r)
        for (int c = 0; c < COLS; ++c)
            grid[r][c] = {r, c, false, {}};

    // --- Indoor layout (three long vertical walls with gaps) ---
    // Left wall
    for (int r = 0; r < ROWS; ++r)
        for (int c = 6; c < 8; ++c)
            grid[r][c].blocked = true;
    for (int c = 6; c < 8; ++c)
        for (int r = 7; r < 10; ++r)
            grid[r][c].blocked = false;  // middle gap

    // Middle wall
    for (int r = 0; r < ROWS; ++r)
        for (int c = 14; c < 16; ++c)
            grid[r][c].blocked = true;
    for (int c = 14; c < 16; ++c)
        for (int r = 3; r < 6; ++r)
            grid[r][c].blocked = false;  // upper gap

    // Right wall
    for (int r = 0; r < ROWS; ++r)
        for (int c = 22; c < 24; ++c)
            grid[r][c].blocked = true;
    for (int c = 22; c < 24; ++c)
        for (int r = 11; r < 14; ++r)
            grid[r][c].blocked = false;  // lower gap

    // --- Create neighbor links ---
    for (int r = 0; r < ROWS; ++r)
        for (int c = 0; c < COLS; ++c)
            for (int dr = -1; dr <= 1; ++dr)
                for (int dc = -1; dc <= 1; ++dc) {
                    if (dr == 0 && dc == 0) continue;
                    int nr = r + dr, nc = c + dc;
                    if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS)
                        grid[r][c].nbrs.push_back(&grid[nr][nc]);
                }

    Agent agent;
    Node* start = &grid[1][1];
    agent.shape.setPosition(toWorld(start));

    Node* goal = &grid[ROWS - 2][COLS - 2];
    std::vector<Node*> path;
    std::vector<sf::CircleShape> crumbs;

    while (window.isOpen()) {
        while (auto e = window.pollEvent()) {
            if (e->is<sf::Event::Closed>()) window.close();

            if (auto m = e->getIf<sf::Event::MouseButtonPressed>()) {
                if (m->button == sf::Mouse::Button::Left) {
                    int c = m->position.x / CELL, r = m->position.y / CELL;
                    if (r >= 0 && r < ROWS && c >= 0 && c < COLS && !grid[r][c].blocked) {
                        goal = &grid[r][c];

                        // ✅ Dynamic start quantization: A* starts from agent's *current position*
                        int agentC = static_cast<int>(agent.shape.getPosition().x / CELL);
                        int agentR = static_cast<int>(agent.shape.getPosition().y / CELL);
                        if (agentR >= 0 && agentR < ROWS && agentC >= 0 && agentC < COLS) {
                            Node* currentNode = &grid[agentR][agentC];
                            if (!currentNode->blocked)
                                path = a_star(currentNode, goal);
                            else
                                path = a_star(start, goal); // fallback
                        }

                        agent.setPath(path);
                        crumbs.clear();
                    }
                }
            }
        }

        agent.update(1.f / 60.f);

        sf::CircleShape crumb(2.f);
        crumb.setOrigin({1.f, 1.f});
        crumb.setFillColor(sf::Color::Yellow);
        crumb.setPosition(agent.shape.getPosition());
        crumbs.push_back(crumb);

        window.clear(sf::Color(240, 240, 240));

        // Draw grid and obstacles
        sf::RectangleShape cell({CELL - 1.f, CELL - 1.f});
        for (int r = 0; r < ROWS; ++r)
            for (int c = 0; c < COLS; ++c) {
                cell.setPosition({(float)c * CELL, (float)r * CELL});
                if (grid[r][c].blocked)
                    cell.setFillColor(sf::Color(0, 255, 0));  // green walls
                else
                    cell.setFillColor(sf::Color(240, 240, 240));  // background
                window.draw(cell);
            }

        // Draw A* path (red)
        for (auto* n : path) {
            sf::CircleShape dot(3.f);
            dot.setOrigin({1.5f, 1.5f});
            dot.setFillColor(sf::Color::Red);
            dot.setPosition(toWorld(n));
            window.draw(dot);
        }

        for (auto& c : crumbs) window.draw(c);
        window.draw(agent.shape);
        window.display();
    }
}
