#include "../../include/graph/GraphWrapper.h"

void UnweightedGraph::addNode(int node_id) {
  graph.addNode(node_id);
}

void UnweightedGraph::addEdge(int u, int v, double weight) {
  graph.addEdge(u, v);
}

// Реализации методов для классов-оберток
void UnweightedGraph::loadFromFile(const std::string& filename) {
    // TODO: Реализовать логику парсинга файла для невзвешенного графа
}

void UnweightedGraph::print() const {
    graph.print();
}



void WeightedGraph::addNode(int node_id) {
  graph.addNode(node_id);
}

void WeightedGraph::addEdge(int u, int v, double weight) {
  graph.addEdge(u, v, weight);
}

void WeightedGraph::loadFromFile(const std::string& filename) {
    // TODO: Реализовать логику парсинга файла для взвешенного графа
}

void WeightedGraph::print() const {
    graph.print();
}
