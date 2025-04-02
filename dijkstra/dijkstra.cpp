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

// Dijkstra 节点结构
struct Node {
    int x, y;
    double g; // 从起点到当前节点的路径代价
    bool operator>(const Node& other) const {
        return g > other.g; // 优先队列基于 g 值排序
    }
};

class Dijkstra {
public:
    Dijkstra(int width, int height, pair<int, int> start, pair<int, int> goal)
        : width(width), height(height), start(start), goal(goal) {
        grid = vector<vector<int>>(width, vector<int>(height, 0)); // 0: 可通行
    }

    void addObstacle(int x, int y) {
        grid[x][y] = 1; // 标记为障碍物
    }

    bool findPath() {
        // 使用最小优先队列来实现Dijkstra算法
        priority_queue<Node, vector<Node>, greater<Node>> openList;
        unordered_map<pair<int, int>, double, hash_pair> g_score;
        unordered_map<pair<int, int>, pair<int, int>, hash_pair> came_from;

        g_score[start] = 0;
        openList.push({start.first, start.second, 0}); // 将起点加入队列

        while (!openList.empty()) {
            Node current = openList.top();
            openList.pop();

            if (make_pair(current.x, current.y) == goal) {
                reconstructPath(came_from); // 找到目标点，重建路径
                return true;
            }

            // 处理当前节点的邻居
            for (auto d : directions) {
                int nx = current.x + d.first, ny = current.y + d.second;
                if (!isValid(nx, ny)) continue; // 检查邻居是否合法

                double tentative_g = g_score[{current.x, current.y}] + 1; // 计算新路径代价
                if (g_score.find({nx, ny}) == g_score.end() || tentative_g < g_score[{nx, ny}]) {
                    g_score[{nx, ny}] = tentative_g; // 更新路径代价
                    openList.push({nx, ny, tentative_g}); // 将邻居加入队列
                    came_from[{nx, ny}] = {current.x, current.y}; // 记录路径
                }
            }
        }
        return false; // 如果队列为空且未找到目标点，表示无路径
    }

    bool addObstacleAndFind(int x, int y) {
        addObstacle(x, y);
        return findPath();
    }

private:
    int width, height;
    pair<int, int> start, goal;
    vector<vector<int>> grid; // 0: 可通行，1: 障碍物

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

int main() {
    pair<int, int> start = {0, 0};
    pair<int, int> goal = {4, 4};
    Dijkstra dijkstra(5, 5, start, goal);

    auto start_time = std::chrono::high_resolution_clock::now();  // Start Time

    dijkstra.addObstacleAndFind(2, 2);
    dijkstra.addObstacleAndFind(2, 3);
    dijkstra.addObstacleAndFind(2, 4);

    auto end_time = std::chrono::high_resolution_clock::now();  // End time
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    std::cout << "Running Time: " << duration.count() << " microseconds. " << std::endl;

    return 0;
}
