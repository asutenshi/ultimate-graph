#pragma once

#include "Ant.h"
#include "../../Way.h"

#include <vector>
#include <memory>
#include <unordered_map>
#include <random>
#include <unordered_set>

template <typename T>
class AntColony {
  const T& graph;
  std::vector<std::unique_ptr<Ant>> ants;
  int maxIterations;
  double evaporationRate;
  double Q;
  double initialPheromone;
  std::unordered_map<int, std::unordered_map<int, double>> pheromones;

  public:
    AntColony(const T& agraph, int iterations = 100,
      double evap = 0.5, double q = 100.0, double initPheromone = 0.1)
      : graph(agraph), maxIterations(iterations),
        evaporationRate(evap), Q(q), initialPheromone(initPheromone) {}
    
    AntColony(const T& agraph, const Ant* antType, int antCount,
      int iterations = 100, double evap = 0.5,
      double q = 100.0, double initPheromone = 0.1)
      : graph(agraph), maxIterations(iterations),
        evaporationRate(evap), Q(q), initialPheromone(initPheromone) {
          for (int i = 0; i < antCount; i++)
            ants.push_back(antType->clone());
        }
    
    AntColony(const T& agraph, std::vector<std::unique_ptr<Ant>> antList,
      int iterations = 100, double evap = 0.5,
      double q = 100.0, double initPheromone = 0.1)
      : graph(agraph), ants(std::move(antList)),
      maxIterations(iterations), evaporationRate(evap),
      Q(q), initialPheromone(initPheromone) {}

    void addAnt(std::unique_ptr<Ant> ant) {
      ants.push_back(std::move(ant));   }
    
    template<typename AntType>
    void addAnts(int count) {
      for (int i = 0; i < count; ++i) {
        ants.push_back(std::make_unique<AntType>());
      }
    }

    size_t getAntCount() const { return ants.size(); }
    
    Way findShortestHamiltonianCycle();

    private:
      void initializePheromones();
      Way constructHamiltonianCycle(int start, const Ant& ant, std::mt19937& rng);
      int selectNextNode(int current, const std::unordered_set<int>& visited, 
                        const Ant& ant, std::mt19937& rng);
      double calculateProbability(double pheromone, double distance, const Ant& ant) const;
      bool isValidHamiltonianCycle(const Way& way, int start);
      void updatePheromones(const std::vector<Way>& allCycles);
      void evaporatePheromones();
      double getPheromone(int from, int to);
      void setPheromone(int from, int to, double value);
};

template <typename T>
void AntColony<T>::initializePheromones() {
  auto& adjList = graph.getAdjList();
  for (const auto& [node, edges] : adjList) {
    for (const auto& edge : edges) {
      setPheromone(node, edge.to, initialPheromone);
    }
  }
}

template <typename T>
double AntColony<T>::getPheromone(int from, int to) {
  if (pheromones.find(from) != pheromones.end() && 
      pheromones[from].find(to) != pheromones[from].end()) {
    return pheromones[from][to];
  }
  return initialPheromone;
}

template <typename T>
void AntColony<T>::setPheromone(int from, int to, double value) {
  pheromones[from][to] = value;
}

template <typename T>
double AntColony<T>::calculateProbability(double pheromone, double distance, const Ant& ant) const {
  double visibility = 1.0 / (distance + 0.0001);
  return std::pow(pheromone, ant.getAlpha()) * std::pow(visibility, ant.getBeta());
}

template <typename T>
bool AntColony<T>::isValidHamiltonianCycle(const Way& way, int start) {
  if (!way.isValid()) return false;
  
  size_t graphSize = graph.getAdjList().size();
  
  // Проверяем, что посещены все вершины + возврат в начало
  if (way.nodes.size() != graphSize + 1) return false;
  
  // Проверяем, что начало и конец совпадают
  if (way.nodes.front() != start || way.nodes.back() != start) return false;
  
  // Проверяем уникальность всех промежуточных вершин
  std::unordered_set<int> visited(way.nodes.begin(), way.nodes.end() - 1);
  if (visited.size() != graphSize) return false;
  
  return true;
}

template <typename T>
Way AntColony<T>::findShortestHamiltonianCycle() {
  if (ants.empty()) {
    throw std::runtime_error("No ants in colony! Add ants before running algorithm.");
  }
  
  auto& adjList = graph.getAdjList();
  if (adjList.empty()) {
    throw std::runtime_error("Graph is empty!");
  }
  
  initializePheromones();
  
  Way bestCycle;
  
  std::random_device rd;
  std::mt19937 rng(rd());
  
  // Начальная вершина - первая вершина графа
  int startVertex = adjList.begin()->first;
  
  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    std::vector<Way> allCycles;
    
    // Каждый муравей строит гамильтонов цикл
    for (const auto& ant : ants) {
      Way cycle = constructHamiltonianCycle(startVertex, *ant, rng);
      
      // Проверяем, что найден валидный гамильтонов цикл
      if (isValidHamiltonianCycle(cycle, startVertex)) {
        allCycles.push_back(cycle);
        
        if (!bestCycle.isValid() || cycle.length < bestCycle.length) {
          bestCycle = cycle;
        }
      }
    }
    
    // Если ни один муравей не нашёл цикл, продолжаем
    if (allCycles.empty()) continue;
    
    // Обновление феромонов
    evaporatePheromones();
    updatePheromones(allCycles);
  }
  
  // Проверка, найден ли цикл
  if (!bestCycle.isValid()) {
    throw std::runtime_error("Hamiltonian cycle not found. The graph may not contain a Hamiltonian cycle.");
  }
  
  return bestCycle;
}

template <typename T>
Way AntColony<T>::constructHamiltonianCycle(int start, const Ant& ant, std::mt19937& rng) {
  std::vector<int> path;
  std::unordered_set<int> visited;
  int current = start;
  size_t graphSize = graph.getAdjList().size();
  
  path.push_back(current);
  visited.insert(current);
  
  // Строим путь через все вершины
  while (visited.size() < graphSize) {
    int next = selectNextNode(current, visited, ant, rng);
    if (next == -1) {
      // Муравей застрял, не может продолжить путь
      return Way(); // возвращаем невалидный Way
    }
    
    path.push_back(next);
    visited.insert(next);
    current = next;
  }
  
  // Проверяем, можем ли вернуться в начальную вершину
  auto& adjList = graph.getAdjList();
  if (adjList.find(current) == adjList.end()) return Way();
  
  bool canReturnToStart = false;
  // double returnEdgeWeight = 0.0;
  for (const auto& edge : adjList.at(current)) {
    if (edge.to == start) {
      canReturnToStart = true;
      // returnEdgeWeight = edge.weight;
      break;
    }
  }
  
  if (canReturnToStart) {
    path.push_back(start); // Замыкаем цикл
    
    // Вычисляем длину пути
    int length = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
      int from = path[i];
      int to = path[i + 1];
      
      const auto& edges = adjList.at(from);
      for (const auto& edge : edges) {
        if (edge.to == to) {
          length += edge.weight;
          break;
        }
      }
    }
    
    Way result;
    result.nodes = path;
    result.length = length;
    return result;
  }
  
  return Way(); // Не удалось замкнуть цикл
}

template <typename T>
int AntColony<T>::selectNextNode(int current, const std::unordered_set<int>& visited, 
                                  const Ant& ant, std::mt19937& rng) {
  auto& adjList = graph.getAdjList();
  if (adjList.find(current) == adjList.end()) return -1;
  
  const auto& edges = adjList.at(current);
  std::vector<int> candidates;
  std::vector<double> probabilities;
  double sum = 0.0;
  
  // Вычисление вероятностей для каждого непосещённого соседа
  for (const auto& edge : edges) {
    if (visited.find(edge.to) == visited.end()) {
      double pheromone = getPheromone(current, edge.to);
      double probability = calculateProbability(pheromone, edge.weight, ant);
      
      candidates.push_back(edge.to);
      probabilities.push_back(probability);
      sum += probability;
    }
  }
  
  if (candidates.empty()) return -1;
  
  // Нормализация вероятностей
  for (auto& p : probabilities) p /= sum;
  
  // Рулетка для выбора следующего узла
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  double rand = dist(rng);
  double cumulative = 0.0;
  
  for (size_t i = 0; i < candidates.size(); ++i) {
    cumulative += probabilities[i];
    if (rand <= cumulative) {
      return candidates[i];
    }
  }
  
  return candidates.back();
}

template <typename T>
void AntColony<T>::updatePheromones(const std::vector<Way>& allCycles) {
  for (const auto& cycle : allCycles) {
    double deltaPheromone = Q / cycle.length;
    
    for (size_t j = 0; j < cycle.nodes.size() - 1; ++j) {
      int from = cycle.nodes[j];
      int to = cycle.nodes[j + 1];
      double current = getPheromone(from, to);
      setPheromone(from, to, current + deltaPheromone);
    }
  }
}

template <typename T>
void AntColony<T>::evaporatePheromones() {
  for (auto& [from, toMap] : pheromones) {
    for (auto& [to, pheromone] : toMap) {
      pheromone *= (1.0 - evaporationRate);
      if (pheromone < initialPheromone * 0.01) {
        pheromone = initialPheromone * 0.01; // минимальное значение
      }
    }
  }
}
