#pragma once

#include <vector>
#include <ostream>

struct Way {
  std::vector<int> nodes;
  int length;
  Way() : length(-1) {}

  bool isValid() const {
    return length >= 0 && !nodes.empty();
  }
};

inline std::ostream& operator<<(std::ostream& os, const Way& way) {
    os << "Length: " << way.length << ", Path: ";
    for (size_t i = 0; i < way.nodes.size(); ++i) {
        os << way.nodes[i];
        if (i + 1 < way.nodes.size()) os << " -> ";
    }
    return os;
}
