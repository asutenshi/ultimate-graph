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
    Way antAlgorithm();
};

template <typename T>
Way GraphAlgorithms<T>::dijkstryAlgorithm(int begin, int end) {
  Dijkstry<T> dijkstra(graph);
  return dijkstra.shortestWay(begin, end);
}

template <typename T>
Way GraphAlgorithms<T>::antAlgorithm() {
  AntColony<T> antColony(graph);
  antColony.template addAnts<BalancedAnt>(10);
  return antColony.findShortestHamiltonianCycle();
}
