#pragma once
#include "../Graph.h"
#include "Dijkstry.h"
#include "AntAlgorithm/AntColony.h"

template <typename T>
class GraphAlgorithms {
  const T& graph;

  public:
    GraphAlgorithms(const T& agraph) : graph(agraph) {}
    Way dijkstryAlgorithm(int begin, int end);

    template<typename AntType>
    Way antAlgorithm(
      int antCount = 10,
      int maxIterations = 100,
      double evaporationRate = 0.1,
      double Q = 100.0,
      double initialPheromone = 0.1,
      int minIterations = 0,
      int stableIterations = 0,
      double eps = 1e-6
    );
};

template <typename T>
Way GraphAlgorithms<T>::dijkstryAlgorithm(int begin, int end) {
  Dijkstry<T> dijkstra(graph);
  return dijkstra.shortestWay(begin, end);
}

template <typename T>
template <typename AntType>
Way GraphAlgorithms<T>::antAlgorithm(
  int antCount,
  int maxIterations,
  double evaporationRate,
  double Q,
  double initialPheromone,
  int minIterations,
  int stableIterations,
  double eps
) {
  AntColony<T> antColony(
    graph, maxIterations, evaporationRate, Q, initialPheromone,
    minIterations, stableIterations, eps
  );
  antColony.template addAnts<AntType>(antCount);
  return antColony.findShortestHamiltonianCycle();
}
