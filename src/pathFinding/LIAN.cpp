#include "../../include/pathFinding/LIAN.h"
#include <limits>
#include <numbers>
#include <fstream>
#include <string>
#include <iostream>

LIAN::LIAN(const std::vector<std::vector<int>>& map)
  : map_(map) {
  rows_ = map.size();
  cols_ = rows_ > 0 ? map[0].size() : 0;
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
        
        // Передаем delta в getNeighbors
        std::vector<Point> neighbors = getNeighbors(current.pos, current.prev, max_turn_angle, delta);
        
        for (const Point& next : neighbors) {
            double newCost = current.cost + distance(current.pos, next);
            
            if (costSoFar.find(next) == costSoFar.end() || newCost < costSoFar[next]) {
                costSoFar[next] = newCost;
                Node nextNode{next, current.pos, newCost, heuristic(next, goal, current.pos)};
                openSet.push(nextNode);
                cameFrom[next] = current.pos;
            }
        }
    }
    
    std::cout << "No path found after " << iterations << " iterations" << std::endl;
    return {};
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
    
    // 8-направленное движение с шагом delta
    std::vector<std::pair<int, int>> directions = {
        {0, delta}, {delta, 0}, {0, -delta}, {-delta, 0},           // прямые
        {delta, delta}, {delta, -delta}, {-delta, delta}, {-delta, -delta}  // диагональные
    };
    
    for (const auto& [dx, dy] : directions) {
        Point next{current.x + dx, current.y + dy};
        
        if (!isValid(next)) {
            continue;
        }
        
        // Проверяем, что весь путь от current до next свободен
        if (!isFreeLine(current, next)) {
            continue;
        }
        
        if (prev != current) {
            if (!isValidAngle(prev, current, next, max_turn_angle)) {
                continue;
            }
        }
        
        neighbors.push_back(next);
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
  
  // Если это стартовая точка, возвращаем только расстояние
  if (current == prev) {
    return distToGoal;
  }
  
  // Штраф за отклонение от прямой линии к цели
  double directDist = distance(prev, goal);
  double stepDist = distance(prev, current);
  double deviation = std::abs((stepDist + distToGoal) - directDist);
  
  // Комбинированная эвристика
  return distToGoal + 3.0 * deviation;
}

std::vector<Point> LIAN::reconstructPath(
    const std::unordered_map<Point, Point, PointHash>& cameFrom,
    const Point& start, const Point& goal) const {
    
    std::vector<Point> path;
    Point current = goal;
    
    // Восстанавливаем путь в обратном порядке
    while (current != start) {
        path.push_back(current);
        auto it = cameFrom.find(current);
        if (it == cameFrom.end()) {
            return {}; // Путь не найден
        }
        current = it->second;
    }
    path.push_back(start);
    
    // Переворачиваем путь
    std::reverse(path.begin(), path.end());
    
    // Добавляем промежуточные точки между каждыми двумя соседними точками
    std::vector<Point> detailedPath;
    
    for (size_t i = 0; i < path.size(); ++i) {
        detailedPath.push_back(path[i]);
        
        // Если есть следующая точка, добавляем промежуточные точки
        if (i + 1 < path.size()) {
            std::vector<Point> linePoints = getLinePoints(path[i], path[i + 1]);
            // Пропускаем первую точку (она уже добавлена) и последнюю (она будет добавлена на следующей итерации)
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

    // Копируем карту
    std::vector<std::vector<int>> resultMap = grid;
    
    // Отмечаем путь числом 2
    for (const auto& point : path) {
        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);
        
        if (y >= 0 && y < static_cast<int>(resultMap.size()) && 
            x >= 0 && x < static_cast<int>(resultMap[0].size())) {
            resultMap[y][x] = 2; // 2 = путь
        }
    }
    
    // Сохраняем в файл
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
