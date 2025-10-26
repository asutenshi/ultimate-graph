#include <iostream>
#include "../include/graph/GraphWrapper.h"
#include "../include/graph/GraphAlgorithms/GraphAlgorithms.h"
#include "../include/pathFinding/LIAN.h"

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

  // ---

  // try {
  //   // WeightedGraph graph("./data/deijkstry.txt");
  //   WeightedGraph graph("./data/1000.txt");
  //   graph.print();

  //   GraphAlgorithms<Graph<WeightedEdge>> algorithms(graph.getGraph());
  //   // std::cout << std::endl << algorithms.dijkstryAlgorithm(1, 9) << std::endl;
  //   std::cout << std::endl << algorithms.dijkstryAlgorithm(0, 874) << std::endl;
  // } catch (const std::exception& ex) {
  //   std::cout << "\nAlgorithm error: " << ex.what() << std::endl;
  // }

//  ---
  
  // try {
  //   WeightedGraph graph("./data/ant_graph_small.txt");
  //   WeightedGraph graph("./data/1000.txt");
  
  //   graph.print();
  
  //   GraphAlgorithms<Graph<WeightedEdge>> algorithms(graph.getGraph());
  //   std::cout << std::endl << algorithms.antAlgorithm() << std::endl;
  // } catch (const std::exception& ex) {
  //     std::cout << "\nAlgorithm error: " << ex.what() << std::endl;
  // }

  //---

  std::vector<std::vector<int>> map = readBinaryMap("./data/binary_map.txt");
  LIAN alg = LIAN(map);

  Point start{165, 304};
  Point goal{1287, 690};
  double maxAngle = 120.0;

  std::vector<Point> path = alg.findPath(start, goal, maxAngle);

  if (!path.empty()) {
      std::cout << "Path found with " << path.size() << " points:" << std::endl;
      for (const auto& p : path) {
          std::cout << "(" << p.x << ", " << p.y << ") ";
      }
      saveMapWithPath(map, path, "./output/map_with_path.txt");
  } else {
      std::cout << "No path found" << std::endl;
  }

  return 0;
}
