#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <string>

struct Point {
    int x, y;
    bool operator==(const Point& other) const { 
        return x == other.x && y == other.y; 
    }
};

struct PointHash {
    std::size_t operator()(const Point& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

struct Node {
    Point pos;
    Point prev;
    double cost;
    double heuristic;
    
    double f() const { return cost + heuristic; }
    
    bool operator>(const Node& other) const {
        return f() > other.f();
    }
};

class LIAN {
public:
    LIAN(const std::vector<std::vector<int>>& map);
    std::vector<Point> findPath(const Point& start, const Point& goal, double max_turn_angle);

private:
    std::vector<std::vector<int>> map_;
    int rows_, cols_;
    
    bool isValid(const Point& p) const;
    bool isFreeLine(const Point& a, const Point& b) const;
    double angleBetween(const Point& a, const Point& b, const Point& c) const;
    bool isValidAngle(const Point& prev, const Point& current, const Point& next, double max_turn_angle) const;
    std::vector<Point> getNeighbors(const Point& current, const Point& prev, double max_turn_angle) const;
    std::vector<Point> getLinePoints(const Point& a, const Point& b) const;
    double distance(const Point& a, const Point& b) const;
    double heuristic(const Point& a, const Point& b) const;
    std::vector<Point> reconstructPath(
        const std::unordered_map<Point, Point, PointHash>& cameFrom,
        const Point& start, const Point& goal) const;
};

std::vector<std::vector<int>> readBinaryMap(const std::string& filename);
bool saveMapWithPath(const std::vector<std::vector<int>>& grid, const std::vector<Point>& path, const std::string& filename);
