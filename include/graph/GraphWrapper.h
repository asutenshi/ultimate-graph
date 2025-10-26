#pragma once
#include "IGraph.h"
#include "Graph.h"
#include <iostream>


class UnweightedGraph : public IGraph {
private:
    Graph<int> graph;

public:
    const Graph<int>& getGraph() const { return graph; }
    void addNode(int node_id) override;
    void addEdge(int u, int v, double weight = 1.0) override;
    void loadFromFile(const std::string& filename) override;
    void print() const override;
};

class WeightedGraph : public IGraph {
private:
    Graph<WeightedEdge> graph;

public:
    WeightedGraph() = default;
    WeightedGraph(const std::string& filename) : graph(filename) {}
    const Graph<WeightedEdge>& getGraph() const { return graph; }
    void addNode(int node_id) override;
    void addEdge(int u, int v, double weight = 1.0) override;
    void loadFromFile(const std::string& filename) override;
    void print() const override;
};
