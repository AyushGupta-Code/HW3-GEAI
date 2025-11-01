#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>
#include <cmath>
#include <SFML/Graphics.hpp>
using namespace std;

struct Edge { int u, v, w; bool directed; };

// --- FONT LOADING ---
static sf::Font loadFont() {
    sf::Font font;
    const char* paths[] = {
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Supplemental/Helvetica.ttc",
        "/Library/Fonts/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    };
    for (auto p : paths)
        if (font.openFromFile(p)) return font;
    cerr << "⚠️ Font not found, labels will be blank.\n";
    return font;
}

// --- ARROW DRAWING ---
static void drawArrow(sf::RenderTarget& tgt, sf::Vector2f p1, sf::Vector2f p2) {
    sf::Vector2f d = p2 - p1;
    float L = hypot(d.x, d.y);
    if (L < 1.f) return;
    d /= L;
    sf::Vector2f left(-d.y, d.x);
    const float s = 10.f;
    sf::VertexArray tri(sf::PrimitiveType::Triangles, 3);
    tri[0].position = p2;
    tri[1].position = p2 - d * s + left * (s * 0.5f);
    tri[2].position = p2 - d * s - left * (s * 0.5f);
    for (int i = 0; i < 3; i++) tri[i].color = sf::Color::Blue;
    tgt.draw(tri);
}

// --- HEURISTIC GENERATOR ---
vector<int> generateHeuristics(const vector<sf::Vector2f>& pos, int goal) {
    vector<int> h(pos.size(), 0);
    for (int i = 0; i < (int)pos.size(); ++i) {
        float dx = pos[i].x - pos[goal].x;
        float dy = pos[i].y - pos[goal].y;
        h[i] = static_cast<int>(sqrt(dx * dx + dy * dy) / 100.0f + 0.5f); // rounded natural number
    }
    return h;
}

int main() {
    // ✅ Clean node list (duplicates + typos removed)
    vector<string> nodes = {
        "University Tower Deck","University Towers","Nelson Hall",
        "David Clark Labs","Governer Scott Courtyard","Killgore Hall",
        "Fox Teaching Lab","Dan Allen Deck","Scott Hall","Bostian Hall",
        "Phytotron","Gardner Hall","Williams Hall","Dabney Hall",
        "Cox Hall","Beaurau of Mines","DH Hill Library","Patterson Hall",
        "Burlington Lab","Polk Hall","Broughton Hall","Ricks Hall",
        "Withers Hall","Riddick Hall","Tomkins Hall","Bell Tower",
        "Winslow Hall","Holladay Hall","Brooks Hall","Kamphoefner Hall",
        "Syme Hall","Welch Hall", "Mann Hall"
    };

    unordered_map<string,int> id;
    for (int i = 0; i < (int)nodes.size(); ++i) id[nodes[i]] = i;

    // ⚠️ Your edges (keep all weights/directions same)
    vector<Edge> E = {
        {id["University Tower Deck"], id["University Towers"], 3, false},
        {id["University Tower Deck"], id["Dan Allen Deck"], 17, false},
        {id["University Towers"], id["Nelson Hall"], 3, false},
        {id["Nelson Hall"], id["Killgore Hall"], 3, true},
        {id["Nelson Hall"], id["David Clark Labs"], 4, true},
        {id["David Clark Labs"], id["Governer Scott Courtyard"], 1, false},
        {id["Governer Scott Courtyard"], id["Killgore Hall"], 4, false},
        {id["Governer Scott Courtyard"], id["Fox Teaching Lab"], 8, false},
        {id["Dan Allen Deck"], id["Fox Teaching Lab"], 5, false},
        {id["Killgore Hall"], id["Scott Hall"], 4, true},
        {id["Scott Hall"], id["Bostian Hall"], 2, false},
        {id["Bostian Hall"], id["Phytotron"], 3, false},
        {id["Bostian Hall"], id["Gardner Hall"], 2, false},
        {id["Gardner Hall"], id["Williams Hall"], 4, false},
        {id["Williams Hall"], id["Dabney Hall"], 2, false},
        {id["Dabney Hall"], id["Cox Hall"], 2, false},
        {id["Cox Hall"], id["Beaurau of Mines"], 3, false},
        {id["Fox Teaching Lab"], id["Beaurau of Mines"], 4, false},
        {id["Scott Hall"], id["DH Hill Library"], 7, true},
        {id["DH Hill Library"], id["Patterson Hall"], 3, true},
        {id["Burlington Lab"], id["Patterson Hall"], 4, true},
        {id["Burlington Lab"], id["Polk Hall"], 4, false},
        {id["Polk Hall"], id["Broughton Hall"], 13, true},
        {id["Beaurau of Mines"], id["Broughton Hall"], 2, false},
        {id["Ricks Hall"], id["Withers Hall"], 8, false},
        {id["Withers Hall"], id["Riddick Hall"], 9, true},
        {id["Broughton Hall"], id["Mann Hall"], 2, false},
        {id["Mann Hall"], id["Riddick Hall"], 3, false},
        {id["Ricks Hall"], id["Tomkins Hall"], 3, false},
        {id["Tomkins Hall"], id["Bell Tower"], 3, false},
        {id["Bell Tower"], id["Winslow Hall"], 6, false},
        {id["Winslow Hall"], id["Holladay Hall"], 4, false},
        {id["Winslow Hall"], id["Brooks Hall"], 3, false},
        {id["Brooks Hall"], id["Kamphoefner Hall"], 3, false},
        {id["Kamphoefner Hall"], id["Syme Hall"], 3, false},
        {id["Brooks Hall"], id["Welch Hall"], 2, false},
        {id["Riddick Hall"], id["Welch Hall"], 5, true},
    };

    // Auto layout for visualization
    vector<sf::Vector2f> pos(nodes.size());
    const float X_STEP = 150, Y_STEP = 80;
    int col = 0, row = 0;
    for (int i = 0; i < (int)nodes.size(); ++i) {
        pos[i] = {100 + (col * X_STEP), 100 + (row * Y_STEP)};
        col++;
        if (col > 6) { col = 0; row++; }
    }

    // 🎯 Compute heuristics relative to Bell Tower
    int goal = id["Bell Tower"];
    vector<int> H = generateHeuristics(pos, goal);

    // Save heuristics to CSV
    {
        ofstream hout("heuristics.csv");
        hout << "Node,Heuristic_to_BellTower\n";
        for (int i = 0; i < (int)nodes.size(); ++i)
            hout << nodes[i] << "," << H[i] << "\n";
    }
    cout << "✓ heuristics.csv written\n";

    // 🖼️ SFML Visualization
    sf::RenderWindow window(sf::VideoMode({1600u, 900u}), "NCSU Graph + Heuristics (Natural Numbers)");
    window.setFramerateLimit(60);
    sf::Font font = loadFont();

    while (window.isOpen()) {
        while (auto ev = window.pollEvent())
            if (ev->is<sf::Event::Closed>()) window.close();

        window.clear(sf::Color(235,245,255));

        // --- Draw edges ---
        for (auto &e : E) {
            sf::Vector2f p1 = pos[e.u];
            sf::Vector2f p2 = pos[e.v];
            sf::VertexArray line(sf::PrimitiveType::Lines, 2);
            line[0].position = p1;
            line[1].position = p2;
            line[0].color = line[1].color = sf::Color::Blue;
            window.draw(line);
            if (e.directed) drawArrow(window, p1, p2);

            sf::Vector2f mid = (p1 + p2) / 2.f;
            sf::Text t(font, to_string(e.w), 16);
            t.setFillColor(sf::Color::Red);
            t.setPosition(sf::Vector2f(mid.x + 5.f, mid.y - 10.f));
            window.draw(t);
        }

        // --- Draw nodes + heuristics ---
        for (int i = 0; i < (int)nodes.size(); ++i) {
            sf::CircleShape c(10);
            c.setOrigin({10,10});
            c.setPosition(pos[i]);
            c.setFillColor(sf::Color(0,200,220));
            window.draw(c);

            sf::Text label(font, nodes[i], 14);
            label.setFillColor(sf::Color::Black);
            label.setPosition(sf::Vector2f(pos[i].x + 15.f, pos[i].y - 10.f));
            window.draw(label);

            sf::Text htxt(font, "h=" + to_string(H[i]), 12);
            htxt.setFillColor(sf::Color::Green);
            htxt.setPosition(sf::Vector2f(pos[i].x - 10.f, pos[i].y + 15.f));
            window.draw(htxt);
        }

        window.display();
    }
}
