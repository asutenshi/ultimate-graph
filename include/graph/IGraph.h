#pragma once
#include <string>

class IGraph {
public:
    virtual ~IGraph() = default;

    virtual void addNode(int node_id) = 0;
    virtual void addEdge(int u, int v, double weight = 1.0) = 0;
    virtual void loadFromFile(const std::string& filename) = 0;
    virtual void print() const = 0;
};
