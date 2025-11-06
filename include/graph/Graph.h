#pragma once
#include <vector>
#include <unordered_map>
#include <type_traits>
#include <stdexcept>
#include "EdgeTypes.h"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>

template <typename T>
class Graph {
private:
  std::unordered_map<int, std::vector<T>> adjList;

public:
  Graph() = default;
  Graph(const std::string& filename);
  void addNode(int node_id);
  void addEdge(int u, int v, double weight = 1.0);
  void removeNode(int node_id);
  void removeEdge(int u, int v);
  const std::unordered_map<int, std::vector<T>>& getAdjList() const;
  void print() const;
};

template<typename T>
Graph<T>::Graph(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }

  std::string line;
  
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue; // Пропускаем пустые строки и комментарии
    
    std::istringstream iss(line);
    int from, to;
    double weight = 1.0;
    
    if constexpr (std::is_same_v<T, int>) {
      // Невзвешенный граф: "from to"
      if (!(iss >> from >> to)) {
        throw std::runtime_error("Invalid format in file for unweighted graph");
      }
    } else if constexpr (std::is_same_v<T, WeightedEdge>) {
      // Взвешенный граф: "from to weight"
      if (!(iss >> from >> to >> weight)) {
        throw std::runtime_error("Invalid format in file for weighted graph");
      }
    } else {
      throw std::runtime_error("Unsupported graph type");
    }
    
    // Добавляем узлы, если их ещё нет
    if (adjList.find(from) == adjList.end()) {
      adjList[from] = {};
    }
    if (adjList.find(to) == adjList.end()) {
      adjList[to] = {};
    }
    
    // Добавляем ребро
    if constexpr (std::is_same_v<T, int>) {
      adjList[from].push_back(to);
      adjList[to].push_back(from);
    } else if constexpr (std::is_same_v<T, WeightedEdge>) {
      adjList[from].push_back(WeightedEdge{to, weight});
      // adjList[to].push_back(WeightedEdge{from, weight});
    }
  }
  
  file.close();
}

template<typename T>
void Graph<T>::addNode(int node_id) {
  if (adjList.find(node_id) != adjList.end()) {
      throw std::runtime_error("Node with this ID already exists.");
  }
  adjList[node_id] = {};
}

template <typename T>
void Graph<T>::addEdge(int u, int v, double weight) {
  if (adjList.find(u) == adjList.end() || adjList.find(v) == adjList.end())
    throw std::runtime_error("Node u or v not found.");
  
  if constexpr (std::is_same_v<T, int>) {
    adjList[u].push_back(v);
    adjList[v].push_back(u);
  } else if constexpr (std::is_same_v<T, WeightedEdge>) {
    adjList[u].push_back(WeightedEdge{v, weight});
  } 
  else {
    throw std::runtime_error("Unsupported type of graph.");
  }
}

template<typename T>
void Graph<T>::removeNode(int node_id) {
  // Удаляем все рёбра, ведущие к node_id
  for (auto& [id, neighbours] : adjList) {
    if constexpr (std::is_same_v<T, int>) {
      neighbours.erase(std::remove(neighbours.begin(), neighbours.end(), node_id), neighbours.end());
    } else if constexpr (std::is_same_v<T, WeightedEdge>) {
      neighbours.erase(std::remove_if(neighbours.begin(), neighbours.end(),
        [node_id](const WeightedEdge& e){ return e.to == node_id; }), neighbours.end());
    }
  }
  // Удаляем саму вершину
  adjList.erase(node_id);
}

template<typename T>
void Graph<T>::removeEdge(int u, int v) {
  if constexpr (std::is_same_v<T, int>) {
    if (adjList.find(u) != adjList.end())
      adjList[u].erase(std::remove(adjList[u].begin(), adjList[u].end(), v), adjList[u].end());
    if (adjList.find(v) != adjList.end())
      adjList[v].erase(std::remove(adjList[v].begin(), adjList[v].end(), u), adjList[v].end());
  } else if constexpr (std::is_same_v<T, WeightedEdge>) {
    if (adjList.find(u) != adjList.end())
      adjList[u].erase(std::remove_if(adjList[u].begin(), adjList[u].end(),
        [v](const WeightedEdge& e){ return e.to == v; }), adjList[u].end());
    // Если граф неориентированный, удаляем и обратное ребро
    if (adjList.find(v) != adjList.end())
      adjList[v].erase(std::remove_if(adjList[v].begin(), adjList[v].end(),
        [u](const WeightedEdge& e){ return e.to == u; }), adjList[v].end());
  }
}

template <typename T>
const std::unordered_map<int, std::vector<T>>& Graph<T>::getAdjList() const {
  return adjList;
}

template<typename T>
void Graph<T>::print() const {
  std::cout << "Graph: " << std::endl;
  for (auto [node_id, neighbours] : adjList) {
    std::cout << "Node " << node_id << " has edges to: ";

    if constexpr (std::is_same_v<T, int>) {
    for (auto neighbour : neighbours)
      std::cout << neighbour << " ";

    std::cout << std::endl;
    } else if constexpr (std::is_same_v<T, WeightedEdge>) {

    for (auto [neighbour, weight] : neighbours)
      std::cout << "(id: " << neighbour << ", w: " << weight << ")";

    std::cout << std::endl;
    }
    else {
      throw std::runtime_error("Unsupported type of graph.");
    }
  }
}
