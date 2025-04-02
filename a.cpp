#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <chrono>

using namespace std;

// 方向向量：右、左、下、上
const vector<pair<int, int>> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

// 哈希函数，支持 pair<int, int> 作为 unordered_map 的 key
struct hash_pair {
    size_t operator()(const pair<int, int>& p) const {
        return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
    }
};

// A* 节点结构
struct Node {
    int x, y;
    double g, h, f;
    bool operator>(const Node& other) const {
        return f > other.f; // 优先队列基于 f 值排序
    }
};

class AStar {
public:
    AStar(int width, int height, pair<int, int> start, pair<int, int> goal)
        : width(width), height(height), start(start), goal(goal) {
        grid = vector<vector<int>>(width, vector<int>(height, 0)); // 0: 可通行
    }

    void addObstacle(int x, int y) {
        grid[x][y] = 1; // 标记为障碍物
    }

    bool findPath() {
        priority_queue<Node, vector<Node>, greater<Node>> openList;
        unordered_map<pair<int, int>, double, hash_pair> g_score;
        unordered_map<pair<int, int>, pair<int, int>, hash_pair> came_from;

        g_score[start] = 0;
        openList.push({start.first, start.second, 0, heuristic(start.first, start.second), heuristic(start.first, start.second)});

        while (!openList.empty()) {
            Node current = openList.top();
            openList.pop();

            if (make_pair(current.x, current.y) == goal) {
                reconstructPath(came_from);
                return true;
            }

            for (auto d : directions) {
                int nx = current.x + d.first, ny = current.y + d.second;
                if (!isValid(nx, ny)) continue;

                double tentative_g = g_score[{current.x, current.y}] + 1;
                if (g_score.find({nx, ny}) == g_score.end() || tentative_g < g_score[{nx, ny}]) {
                    g_score[{nx, ny}] = tentative_g;
                    double h = heuristic(nx, ny);
                    openList.push({nx, ny, tentative_g, h, tentative_g + h});
                    came_from[{nx, ny}] = {current.x, current.y};
                }
            }
        }
        return false; // 无路径
    }

    bool addObstacleAndFind(int x, int y) {
        addObstacle(x, y);
        return findPath();
    }

private:
    int width, height;
    pair<int, int> start, goal;
    vector<vector<int>> grid;

    double heuristic(int x, int y) {
        return abs(x - goal.first) + abs(y - goal.second); // 曼哈顿距离
    }

    bool isValid(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height && grid[x][y] == 0;
    }

    void reconstructPath(unordered_map<pair<int, int>, pair<int, int>, hash_pair>& came_from) {
        pair<int, int> current = goal;
        vector<pair<int, int>> path;
        while (current != start) {
            path.push_back(current);
            current = came_from[current];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());

        // 打印路径
        for (const auto& p : path) {
            cout << "(" << p.first << ", " << p.second << ") -> ";
        }
        cout << "Goal reached!" << endl;
    }
};

// 主函数
int main() {
    pair<int, int> start = {0, 0};
    pair<int, int> goal = {4, 4};
    AStar astar(5, 5, start, goal);

    auto start_time = std::chrono::high_resolution_clock::now();  // Start Time

    astar.addObstacleAndFind(2, 2);
    astar.addObstacleAndFind(2, 3);
    astar.addObstacleAndFind(2, 4);

    auto end_time = std::chrono::high_resolution_clock::now();  // End time
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cout << "Running Time: " << duration.count() << " microseconds. " << std::endl;

    return 0;
}
