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

// 定义一个节点结构
struct Node {
    int x, y;
    double g, rhs;
    double heuristic;
    bool operator>(const Node& other) const {
        return (min(g, rhs) + heuristic) > (min(other.g, other.rhs) + other.heuristic);
    }
};

class DStarLite {
public:
    DStarLite(int width, int height, pair<int, int> start, pair<int, int> goal)
        : width(width), height(height), start(start), goal(goal) {
        initialize();
    }

    void initialize() {
        grid = vector<vector<int>>(width, vector<int>(height, 0)); // 初始化地图，0 代表可通行
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                Node node = {i, j, numeric_limits<double>::infinity(), numeric_limits<double>::infinity(), heuristic(i, j)};
                nodes[{i, j}] = node;
            }
        }
        nodes[goal].rhs = 0;
        openList.push(nodes[goal]); // 目标节点加入 openList
    }

    void computeShortestPath() {
        while (!openList.empty()) {
            Node u = openList.top();
            openList.pop();

            if (u.g == u.rhs) continue;

            nodes[{u.x, u.y}].g = u.rhs;

            // 更新邻居
            for (auto d : directions) {
                int nx = u.x + d.first, ny = u.y + d.second;
                if (!isValid(nx, ny)) continue;
                updateVertex(nx, ny);
            }
        }
    }

    void updateVertex(int x, int y) {
        if (make_pair(x, y) != goal) {
            double min_rhs = numeric_limits<double>::infinity();
            for (auto d : directions) {
                int nx = x + d.first, ny = y + d.second;
                if (isValid(nx, ny)) {
                    min_rhs = min(min_rhs, nodes[{nx, ny}].g + 1);
                }
            }
            nodes[{x, y}].rhs = min_rhs;
        }
        openList.push(nodes[{x, y}]);
    }

    void addObstacle(int x, int y) {
        grid[x][y] = 1; // 标记为障碍物
        for (auto d : directions) {
            int nx = x + d.first, ny = y + d.second;
            if (isValid(nx, ny)) {
                updateVertex(nx, ny); // 只更新受影响的邻居
            }
        }
        computeShortestPath(); // 重新计算受影响的路径
    }

    void printPath() {
        pair<int, int> current = start;
        while (current != goal) {
            cout << "(" << current.first << ", " << current.second << ") -> ";
            double min_g = numeric_limits<double>::infinity();
            pair<int, int> next_step = current;

            for (auto d : directions) {
                int nx = current.first + d.first, ny = current.second + d.second;
                if (isValid(nx, ny) && nodes[{nx, ny}].g < min_g) {
                    min_g = nodes[{nx, ny}].g;
                    next_step = {nx, ny};
                }
            }
            if (next_step == current) break;
            current = next_step;
        }
        cout << "Goal reached!" << endl;
    }

private:
    int width, height;
    pair<int, int> start, goal;
    vector<vector<int>> grid; // 0: 可通行, 1: 障碍
    unordered_map<pair<int, int>, Node, hash_pair> nodes;
    priority_queue<Node, vector<Node>, greater<Node>> openList;

    double heuristic(int x, int y) {
        return abs(x - goal.first) + abs(y - goal.second);
    }

    bool isValid(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height && grid[x][y] == 0;
    }
};

// 主函数
int main() {
    pair<int, int> start = {0, 0};
    pair<int, int> goal = {4, 4};
    DStarLite dstar(5, 5, start, goal);

    auto start_time = std::chrono::high_resolution_clock::now();  // Start Time

    dstar.computeShortestPath();
    dstar.printPath();

    cout << "Adding obstacle at (2,2)..." << endl;
    dstar.addObstacle(2, 2);  // 添加障碍物，触发局部路径更新
    dstar.printPath();

    cout << "Adding obstacle at (2,4)..." << endl;
    dstar.addObstacle(2, 4);  // 添加障碍物，触发局部路径更新
    dstar.printPath();

    auto end_time = std::chrono::high_resolution_clock::now();  // End time
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cout << "Running Time: " << duration.count() << " microseconds. " << std::endl;

    return 0;
}
