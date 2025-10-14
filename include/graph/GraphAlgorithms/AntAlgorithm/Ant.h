#pragma once

#include <memory>

class Ant {
  protected:
    double alpha;
    double beta;
  
  public:
    Ant(double a = 1.0, double b = 2.0) : alpha(a), beta(b) {}
    virtual ~Ant() = default;
    virtual std::unique_ptr<Ant> clone() const = 0;

    double getAlpha() const { return alpha; }
    double getBeta() const { return beta; }
};

// настраиваемый муравей
class CustomAnt : public Ant {
  public:
    CustomAnt(double a, double b) : Ant(a, b) {}
    std::unique_ptr<Ant> clone() const override {
      return std::make_unique<CustomAnt>(*this);
    }
};

class BalancedAnt : public Ant {
  public:
    BalancedAnt() : Ant(1.0, 2.0) {}
    std::unique_ptr<Ant> clone() const override {
      return std::make_unique<BalancedAnt>(*this);
    }
};

// больше внимания феромонам
class AggressiveAnt : public Ant {
  public:
    AggressiveAnt() : Ant(2.0, 1.0) {}
    std::unique_ptr<Ant> clone() const override {
      return std::make_unique<AggressiveAnt>(*this);
    }
};

// больше внимания расстоянию
class GreedyAnt : public Ant {
  public:
    GreedyAnt() : Ant(0.5, 3.0) {}
    std::unique_ptr<Ant> clone() const override {
      return std::make_unique<GreedyAnt>(*this);
    }
};
