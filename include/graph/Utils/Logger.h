#pragma once
#include <fstream>
#include <string>
#include <stdexcept>

inline void logToFile(const std::string& filename, const std::string& message) {
  std::ofstream out(filename, std::ios::app);
  
  if (out.is_open()) {
    out << message << std::endl;
  }
  else {
    throw std::runtime_error("Cannot open log file: " + filename);
  }
  out.close();
}

inline void clearFile(const std::string& filename) {
  std::ofstream out(filename, std::ios::out | std::ios::trunc);
  if (out.is_open()) {
    out.close();
  }
  else {
    throw std::runtime_error("Cannot open log file: " + filename);
  }
}
