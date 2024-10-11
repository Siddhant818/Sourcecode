#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

using namespace std;

struct Node {
    int x, y;
    double f, g, h;  // f = g + h (total cost), g = cost from start, h = heuristic (estimated cost to goal)
    Node* parent;    // Parent node for path reconstruction

    Node(int x, int y, double g = 0.0, double h = 0.0, Node* parent = nullptr)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}

    // Comparator to prioritize nodes with smaller f values (min-heap)
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// Utility function to check if a given position is within the grid
bool isValid(int x, int y, int rows, int cols) {
    return (x >= 0 && x < rows && y >= 0 && y < cols);
}

// Heuristic function: Manhattan Distance
double heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// A* algorithm function
vector<pair<int, int>> aStar(vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) {
    int rows = grid.size();
    int cols = grid[0].size();

    // Directions for moving in 4 possible ways: up, down, left, right
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    // Priority queue (min-heap) to store nodes, ordered by the lowest f value
    priority_queue<Node, vector<Node>, greater<Node>> pq;

    // Start node
    Node* startNode = new Node(start.first, start.second, 0.0, heuristic(start.first, start.second, goal.first, goal.second));
    pq.push(*startNode);

    // 2D vector to track visited nodes
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));

    // 2D vector to store the cost from the start to each position
    vector<vector<double>> cost(rows, vector<double>(cols, INFINITY));
    cost[start.first][start.second] = 0.0;

    // Loop until the priority queue is empty
    while (!pq.empty()) {
        Node current = pq.top(); // Get the node with the lowest f value
        pq.pop();

        int x = current.x;
        int y = current.y;

        // If we reached the goal, reconstruct the path
        if (x == goal.first && y == goal.second) {
            vector<pair<int, int>> path;
            Node* temp = &current;

            // Backtrack from goal to start to construct the path
            while (temp != nullptr) {
                path.push_back({temp->x, temp->y});
                temp = temp->parent;
            }
            reverse(path.begin(), path.end());  // Reverse the path to get it from start to goal
            return path;
        }

        // Mark the current node as visited
        visited[x][y] = true;

        // Explore the 4 possible neighbors
        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];

            if (isValid(newX, newY, rows, cols) && grid[newX][newY] == 0 && !visited[newX][newY]) {
                double newCost = cost[x][y] + 1.0;  // Assume each step has a cost of 1

                // If a shorter path to the neighbor is found
                if (newCost < cost[newX][newY]) {
                    cost[newX][newY] = newCost;
                    double h = heuristic(newX, newY, goal.first, goal.second);  // Heuristic estimate to goal
                    pq.push(Node(newX, newY, newCost, h, new Node(x, y, current.g, current.h, current.parent)));  // Push neighbor node to priority queue
                }
            }
        }
    }

    // If the goal cannot be reached, return an empty path
    return {};
}

int main() {
    int rows, cols;
    cout << "Enter the number of rows and columns of the grid: ";
    cin >> rows >> cols;

    vector<vector<int>> grid(rows, vector<int>(cols));

    cout << "Enter the grid (0 for free cell, 1 for obstacle):\n";
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            cin >> grid[i][j];
        }
    }

    pair<int, int> start, goal;
    cout << "Enter the start coordinates (row and column): ";
    cin >> start.first >> start.second;
    cout << "Enter the goal coordinates (row and column): ";
    cin >> goal.first >> goal.second;

    // Run A* algorithm
    vector<pair<int, int>> path = aStar(grid, start, goal);

    if (!path.empty()) {
        cout << "Shortest path:\n";
        for (const auto& p : path) {
            cout << "(" << p.first << ", " << p.second << ") ";
        }
        cout << endl;
    } else {
        cout << "No path found from start to goal.\n";
    }

    return 0;
}
