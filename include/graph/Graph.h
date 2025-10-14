#pragma once
#include <vector>
#include <unordered_map>
#include <type_traits>
#include <stdexcept>
#include "EdgeTypes.h"
#include <iostream>

template <typename T>
class Graph {
private:
  std::unordered_map<int, std::vector<T>> adjList;

public:
  void addNode(int node_id);
  void addEdge(int u, int v, double weight = 1.0);
  const std::unordered_map<int, std::vector<T>>& getAdjList() const;
  void print() const;
};

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
