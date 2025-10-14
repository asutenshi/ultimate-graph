#include <iostream>
#include <memory>
#include "../include/graph/GraphWrapper.h"
#include "../include/graph/GraphAlgorithms/GraphAlgorithms.h"

// Главная функция с меню
int main() {
  // std::cout << "Select graph type:\n1: Unweighted\n2: Weighted\n";
  // int graphType = 0;
  // std::cin >> graphType;

  // std::unique_ptr<IGraph> graph;

  // if (graphType == 1) {
  //     graph = std::make_unique<UnweightedGraph>();
  // } else if (graphType == 2) {
  //     graph = std::make_unique<WeightedGraph>();
  // } else {
  //     std::cerr << "Invalid type!\n";
  //     return 1;
  // }

  // std::cout << "Enter filename: ";
  // std::string filename;
  // std::cin >> filename;
  // graph->loadFromFile(filename);

  // std::cout << "Select algorithm:\n1: BFS\n2: Dijkstra\n";
  // int algoId = 0;
  // std::cin >> algoId;
  // graph->runAlgorithm(algoId);

  // std::unique_ptr<IGraph> graph = std::make_unique<WeightedGraph>();
  // WeightedGraph graph;

  // graph.addNode(1);
  // graph.addNode(2);
  // graph.addNode(3);
  // graph.addNode(4);
  // graph.addNode(5);
  // graph.addNode(6);
  // graph.addNode(7);
  // graph.addNode(8);
  // graph.addNode(9);

  // graph.addEdge(1, 2, 10);
  // graph.addEdge(1, 3, 6);
  // graph.addEdge(1, 4, 8);
  // graph.addEdge(2, 4, 5);
  // graph.addEdge(2, 7, 11);
  // graph.addEdge(3, 5, 3);
  // graph.addEdge(4, 7, 12);
  // graph.addEdge(4, 6, 7);
  // graph.addEdge(4, 5, 5);
  // graph.addEdge(5, 6, 9);
  // graph.addEdge(5, 9, 12);
  // graph.addEdge(6, 8, 8);
  // graph.addEdge(6, 9, 10);
  // graph.addEdge(7, 6, 4);
  // graph.addEdge(7, 8, 6);
  // graph.addEdge(8, 9, 15);

  // graph.print();

  // GraphAlgorithms<Graph<WeightedEdge>> algorithms(graph.getGraph());
  // std::cout << std::endl << algorithms.dijkstryAlgorithm(1, 9) << std::endl;

  // Добавление узлов (1='a', 2='b', 3='c', 4='d', 5='f', 6='g')

  WeightedGraph graph;
  graph.addNode(1);
  graph.addNode(2);
  graph.addNode(3);
  graph.addNode(4);
  graph.addNode(5);
  graph.addNode(6);

  // Добавление ребер (Source, Destination, Weight)

  graph.addEdge(1, 2, 3);
  graph.addEdge(1, 5, 1);

  graph.addEdge(2, 1, 3);
  graph.addEdge(2, 3, 8);
  graph.addEdge(2, 6, 3);

  graph.addEdge(3, 2, 3);
  graph.addEdge(3, 4, 1);
  graph.addEdge(3, 6, 1);

  graph.addEdge(4, 3, 8);
  graph.addEdge(4, 5, 1);

  graph.addEdge(5, 4, 3);
  graph.addEdge(5, 1, 3);

  graph.addEdge(6, 1, 3);
  graph.addEdge(6, 2, 3);
  graph.addEdge(6, 3, 3);
  graph.addEdge(6, 4, 5);
  graph.addEdge(6, 5, 4);

  graph.print();

  GraphAlgorithms<Graph<WeightedEdge>> algorithms(graph.getGraph());

  try {
    std::cout << std::endl << algorithms.antAlgorithm() << std::endl;
  } catch (const std::exception& ex) {
      std::cout << "\nAlgorithm error: " << ex.what() << std::endl;
  }

  return 0;
}
