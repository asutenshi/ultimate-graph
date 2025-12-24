#include "../../include/pathFinding/LIAN.h"
#include <limits>
#include <numbers>
#include <fstream>
#include <string>
#include <iostream>
#include <unordered_set>
#include <array>

LIAN::LIAN(const std::vector<std::vector<int>>& map)
  : map_(map) {
  rows_ = map.size();
  cols_ = rows_ > 0 ? map[0].size() : 0;
}

LIAN::LIAN(const std::vector<std::vector<int>>& grid, const std::string& configFile)
    : map_(grid) {
    rows_ = map_.size();
    cols_ = rows_ > 0 ? map_[0].size() : 0;
    loadConfig(configFile);
}

std::vector<Point> LIAN::findPath() {
    return findPath(start_, goal_, maxAngle_, delta_);
}

std::string LIAN::readMapPath(const std::string& configFile) {
    std::ifstream file(configFile);
    if (!file.is_open()) return "";

    std::string line;
    while (std::getline(file, line)) {
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        if (line[first] == '[' || line[first] == '#') continue;

        size_t eqPos = line.find('=');
        if (eqPos != std::string::npos) {
            std::string key = line.substr(first, eqPos - first);
            std::string value = line.substr(eqPos + 1);
            
            while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) key.pop_back();
            size_t valStart = value.find_first_not_of(" \t");
            if (valStart != std::string::npos) value = value.substr(valStart);
            while (!value.empty() && (value.back() == ' ' || value.back() == '\t' || value.back() == '\r')) value.pop_back();

            for (char &c : key) c = std::tolower(c);

            if (key == "mapfile" || key == "map") {
                return value;
            }
        }
    }
    return "";
}

void LIAN::loadConfig(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open config file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        size_t last = line.find_last_not_of(" \t\r\n");
        std::string trimmed = line.substr(first, (last - first + 1));

        if (trimmed.empty() || trimmed[0] == '[' || trimmed[0] == '#') continue;

        size_t eqPos = trimmed.find('=');
        if (eqPos != std::string::npos) {
            std::string key = trimmed.substr(0, eqPos);
            std::string value = trimmed.substr(eqPos + 1);

            while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) key.pop_back();
            size_t valStart = value.find_first_not_of(" \t");
            if (valStart != std::string::npos) value = value.substr(valStart);

            for (char &c : key) c = std::tolower(c);

            try {
                if (key == "start" || key == "goal") {
                    size_t commaPos = value.find(',');
                    if (commaPos != std::string::npos) {
                        int x = std::stoi(value.substr(0, commaPos));
                        int y = std::stoi(value.substr(commaPos + 1));
                        if (key == "start") {
                            start_ = {x, y};
                            std::cout << "DEBUG: Loaded start = " << x << ", " << y << std::endl;
                        } else {
                            goal_ = {x, y};
                            std::cout << "DEBUG: Loaded goal = " << x << ", " << y << std::endl;
                        }
                    }
                } else if (key == "maxangle") {
                    maxAngle_ = std::stod(value);
                    std::cout << "DEBUG: Loaded maxAngle = " << maxAngle_ << std::endl;
                } else if (key == "delta") {
                    delta_ = std::stoi(value);
                    std::cout << "DEBUG: Loaded delta = " << delta_ << std::endl;
                } else if (key == "outputfile" || key == "output") {
                    outputFile_ = value;
                    std::cout << "DEBUG: Loaded outputFile = " << outputFile_ << std::endl;
                }
            } catch (...) {
                std::cerr << "Error parsing config line: " << trimmed << std::endl;
            }
        }
    }
    file.close();
}

std::vector<Point> LIAN::findPath(const Point& start, const Point& goal, double max_turn_angle, int delta) {
    if (!isValid(start) || !isValid(goal)) {
        std::cout << "Invalid start or goal!" << std::endl;
        return {};
    }
    
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_map<Point, double, PointHash> costSoFar;
    std::unordered_map<Point, Point, PointHash> cameFrom;
    
    Node startNode{start, start, 0.0, heuristic(start, goal, start)};
    openSet.push(startNode);
    costSoFar[start] = 0.0;
    cameFrom[start] = start;
    
    int iterations = 0;
    
    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();
        
        iterations++;
        if (iterations % 1000 == 0) {
            std::cout << "Iteration: " << iterations 
                      << ", Queue size: " << openSet.size() 
                      << ", Current: (" << current.pos.x << ", " << current.pos.y << ")" 
                      << ", Cost: " << current.cost << std::endl;
        }
        
        if (costSoFar.count(current.pos) && current.cost > costSoFar[current.pos]) {
            continue;
        }

        if (current.prev != current.pos) {
            Point grandParent = current.prev;
            if (cameFrom.count(current.prev)) {
                grandParent = cameFrom.at(current.prev);
            }
            
            if (grandParent != current.prev) {
                if (!isValidAngle(grandParent, current.prev, current.pos, max_turn_angle)) {
                    continue;
                }
            }
        }
        
        if (current.pos == goal) {
            std::cout << "Goal reached! Iterations: " << iterations << std::endl;
            return reconstructPath(cameFrom, start, goal);
        }
        
        if (isFreeLine(current.pos, goal)) {
            if (current.prev == current.pos || isValidAngle(current.prev, current.pos, goal, max_turn_angle)) {
                double newCost = current.cost + distance(current.pos, goal);
                
                if (costSoFar.find(goal) == costSoFar.end() || newCost < costSoFar[goal]) {
                    costSoFar[goal] = newCost;
                    cameFrom[goal] = current.pos;
                    
                    std::vector<Point> pathToCurrentPos = reconstructPath(cameFrom, start, current.pos);
                    std::vector<Point> linePoints = getLinePoints(current.pos, goal);
                    pathToCurrentPos.insert(pathToCurrentPos.end(), linePoints.begin() + 1, linePoints.end());
                    
                    std::cout << "Found direct path! Total iterations: " << iterations << std::endl;
                    return pathToCurrentPos;
                }
            }
        }
        
        std::vector<Point> neighbors = getNeighbors(current.pos, current.prev, max_turn_angle, delta);
        
        for (const Point& next : neighbors) {
            Point parent = current.pos;
            double costToParent = current.cost;

            bool canShortcut = false;
            
            if (current.prev != current.pos && isFreeLine(current.prev, next)) {
                Point grandParent = current.prev;
                if (cameFrom.count(current.prev)) {
                    grandParent = cameFrom.at(current.prev);
                }

                if (isValidAngle(grandParent, current.prev, next, max_turn_angle)) {
                    canShortcut = true;
                }
            }

            if (canShortcut) {
                parent = current.prev;
                costToParent = costSoFar[parent];
            }

            double newCost = costToParent + distance(parent, next);
            
            if (costSoFar.find(next) == costSoFar.end() || newCost < costSoFar[next]) {
                costSoFar[next] = newCost;
                Node nextNode{next, parent, newCost, heuristic(next, goal, parent)};
                openSet.push(nextNode);
                cameFrom[next] = parent;
            }
        }
    }
    
    std::cout << "No path found after " << iterations << " iterations" << std::endl;
    return {};
}

bool savePathToCSV(const std::vector<Point>& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file for CSV: " << filename << std::endl;
        return false;
    }

    file << "x,y,angle\n";

    for (size_t i = 0; i < path.size(); ++i) {
        double angle = 0.0;

        if (i > 0 && i < path.size() - 1) {
            const Point& prev = path[i - 1];
            const Point& curr = path[i];
            const Point& next = path[i + 1];

            double dx1 = curr.x - prev.x;
            double dy1 = curr.y - prev.y;
            
            double dx2 = next.x - curr.x;
            double dy2 = next.y - curr.y;

            double dot = dx1 * dx2 + dy1 * dy2;
            double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

            if (mag1 > 0 && mag2 > 0) {
                double cosAngle = dot / (mag1 * mag2);
                cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
                angle = std::acos(cosAngle) * 180.0 / std::numbers::pi;
            }
        }

        file << path[i].x << "," << path[i].y << "," << angle << "\n";
    }

    file.close();
    std::cout << "Path CSV saved to: " << filename << std::endl;
    return true;
}


// Метод для получения точек по прямой линии
std::vector<Point> LIAN::getLinePoints(const Point& a, const Point& b) const {
    std::vector<Point> points;
    int dx = std::abs(b.x - a.x);
    int dy = std::abs(b.y - a.y);
    int sx = a.x < b.x ? 1 : -1;
    int sy = a.y < b.y ? 1 : -1;
    int err = dx - dy;
    
    Point current = a;
    while (true) {
        points.push_back(current);
        
        if (current == b) {
            break;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            current.x += sx;
        }
        if (e2 < dx) {
            err += dx;
            current.y += sy;
        }
    }
    
    return points;
}

bool LIAN::isValid(const Point& p) const {
  return p.y >= 0 && p.y < rows_ && p.x >= 0 && p.x < cols_ && map_[p.y][p.x] == 0;
}

bool LIAN::isFreeLine(const Point& a, const Point& b) const {
  int dx = std::abs(b.x - a.x);
  int dy = std::abs(b.y - a.y);
  int sx = a.x < b.x ? 1 : -1;
  int sy = a.y < b.y ? 1 : -1;
  int err = dx - dy;
  
  Point current = a;
  while (true) {
      if (!isValid(current)) {
          return false;
      }
      
      if (current == b) {
          return true;
      }
      
      int e2 = 2 * err;
      if (e2 > -dy) {
          err -= dy;
          current.x += sx;
      }
      if (e2 < dx) {
          err += dx;
          current.y += sy;
      }
  }
}

double LIAN::angleBetween(const Point& a, const Point& b, const Point& c) const {
  double dx1 = b.x - a.x;
  double dy1 = b.y - a.y;
  double dx2 = c.x - b.x;
  double dy2 = c.y - b.y;
  
  double dot = dx1 * dx2 + dy1 * dy2;
  double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
  
  if (mag1 == 0 || mag2 == 0) return 0;
  
  double cosAngle = dot / (mag1 * mag2);
  cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
  
  return std::acos(cosAngle) * 180.0 / std::numbers::pi;
}

bool LIAN::isValidAngle(const Point& prev, const Point& current, const Point& next, double max_turn_angle) const {
  if (prev == current) return true;
  double angle = angleBetween(prev, current, next);
  return angle <= max_turn_angle;
}

std::vector<Point> LIAN::getNeighbors(const Point& current, const Point& prev, double max_turn_angle, int delta) const {
    std::vector<Point> neighbors;
    std::unordered_set<Point, PointHash> uniquePoints;
    
    int x = 0;
    int y = delta;
    int d = 3 - 2 * delta;
    
    auto addSymmetricPoints = [&](int dx, int dy) {
        std::array<std::pair<int, int>, 8> symmetricPoints = {{
            {dx, dy}, {-dx, dy}, {dx, -dy}, {-dx, -dy},
            {dy, dx}, {-dy, dx}, {dy, -dx}, {-dy, -dx}
        }};
        
        for (const auto& [offsetX, offsetY] : symmetricPoints) {
            if (offsetX == 0 && offsetY == 0) continue;
            
            Point next{current.x + offsetX, current.y + offsetY};
            
            // Быстрая проверка уникальности через unordered_set
            if (uniquePoints.count(next)) continue;
            
            if (!isValid(next)) continue;
            
            if (!isFreeLine(current, next)) continue;
            
            if (prev != current && !isValidAngle(prev, current, next, max_turn_angle)) {
                continue;
            }
            
            uniquePoints.insert(next);
            neighbors.push_back(next);
        }
    };
    
    while (x <= y) {
        addSymmetricPoints(x, y);
        
        if (d < 0) {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
    
    return neighbors;
}

double LIAN::distance(const Point& a, const Point& b) const {
  int dx = b.x - a.x;
  int dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

double LIAN::heuristic(const Point& current, const Point& goal, const Point& prev) const {
  double distToGoal = distance(current, goal);
  
  if (current == prev) {
    return distToGoal;
  }
  
  double directDist = distance(prev, goal);
  double stepDist = distance(prev, current);
  double deviation = std::abs((stepDist + distToGoal) - directDist);
  
  return distToGoal + 3.0 * deviation;
}

std::vector<Point> LIAN::reconstructPath(
    const std::unordered_map<Point, Point, PointHash>& cameFrom,
    const Point& start, const Point& goal) const {
    
    std::vector<Point> path;
    Point current = goal;
    
    while (current != start) {
        path.push_back(current);
        auto it = cameFrom.find(current);
        if (it == cameFrom.end()) {
            return {};
        }
        current = it->second;
    }
    path.push_back(start);
    
    std::reverse(path.begin(), path.end());
    savePathToCSV(path, outputFile_);
    
    std::vector<Point> detailedPath;
    
    for (size_t i = 0; i < path.size(); ++i) {
        detailedPath.push_back(path[i]);
        
        if (i + 1 < path.size()) {
            std::vector<Point> linePoints = getLinePoints(path[i], path[i + 1]);
            for (size_t j = 1; j < linePoints.size() - 1; ++j) {
                detailedPath.push_back(linePoints[j]);
            }
        }
    }
    
    return detailedPath;
}

std::vector<std::vector<int>> readBinaryMap(const std::string& filename) {
  std::vector<std::vector<int>> map;
  std::ifstream fin(filename);
  std::string line;
  while (std::getline(fin, line)) {
      std::vector<int> row;
      for (char c : line) {
          if (c == '0' || c == '1')
              row.push_back(c - '0');
      }
      if (!row.empty())
          map.push_back(row);
  }
  return map;
}

bool saveMapWithPath(const std::vector<std::vector<int>>& grid, const std::vector<Point>& path, const std::string& filename) {
    if (grid.empty() || grid[0].empty()) {
        std::cerr << "Empty map!" << std::endl;
        return false;
    }

    std::vector<std::vector<int>> resultMap = grid;
    
    for (const auto& point : path) {
        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);
        
        if (y >= 0 && y < static_cast<int>(resultMap.size()) && 
            x >= 0 && x < static_cast<int>(resultMap[0].size())) {
            resultMap[y][x] = 2; // 2 = путь
        }
    }
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return false;
    }
    
    for (const auto& row : resultMap) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << " ";
            }
        }
        file << "\n";
    }
    
    file.close();
    std::cout << "Map with path saved to: " << filename << std::endl;
    return true;
}
