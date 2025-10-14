#pragma once
#include <vector>
#include <map>
#include <queue>
#include <algorithm>
#include "../Graph.h"
#include "../Way.h"

struct MarkedNode {
  int node;
  int mark;
  int prev;

  MarkedNode(int anode=0, int amark=0, int aprev=0):
    node(anode), mark(amark), prev(aprev) {}

  bool operator<(const MarkedNode& other) const {
    return mark > other.mark;
  }
};



template <typename T>
class Dijkstry {
  const T& graph;

  public:
    // TODO: написать конструктор с проверкой
    Dijkstry(const T& agraph) : graph(agraph) {}
    Way shortestWay(int begin, int end);
    Way unroll(std::map<int, MarkedNode> visited, int begin, int curr);
};

template <typename T>
Way Dijkstry<T>::shortestWay(int begin, int end) {
  std::priority_queue<MarkedNode> nodes;
  nodes.push(MarkedNode(begin, 0, 0));
  std::map<int, MarkedNode> visited;

  while (!nodes.empty()) {
    MarkedNode next = nodes.top();
    nodes.pop();

    if (visited.count(next.node)) {
      continue;
    }
    visited[next.node] = next;

    if (end == next.node)
      return unroll(visited, begin, end);
    
    for (const WeightedEdge& neighbourEdge : graph.getAdjList().at(next.node)) {
      int weight = neighbourEdge.weight + next.mark;

      if (visited.find(neighbourEdge.to) == visited.end())
        nodes.push(MarkedNode(neighbourEdge.to, weight, next.node));
    }
  }
  return Way();
}

template <typename T>
Way Dijkstry<T>::unroll(std::map<int, MarkedNode> visited, int begin, int curr) {
  Way way;
  way.length = visited[curr].mark;

  while (curr != begin) {
    way.nodes.insert(way.nodes.begin(), curr);
    curr = visited[curr].prev;
  }
  way.nodes.insert(way.nodes.begin(), begin);
  return way;
}
