
#include "word_counting/word_counting.hpp"

#include <iostream>
#include <stdexcept>
#include <vector>
#include <random>


std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(1.0, 2.0);


namespace word_counting {

struct Pose {
  double x;
  double y;
  Pose(double x_, double y_) : x(x_), y(y_) {}
  void update(double dt)
  {
    double dx = dis(gen);
    double dy = dis(gen);
    x += dx * dt;
    y += dy * dt;
  }
  Pose operator-(const Pose& other) const
  {
    return {x - other.x, y - other.y};
  }
  double distance() const
  {
    return std::sqrt(x * x + y * y);
  }
};

struct Trajectory {
  Pose pose;
  std::vector<Pose> poses;
  Trajectory(double x0, double y0, size_t nSteps) : pose(x0, y0), poses(1,
                                                                        pose)
  {
    double dt = 10.0 / static_cast<double>(nSteps);
    for (size_t i = 0; i < nSteps; ++i) {
      pose.update(dt);
      poses.push_back(pose);
    }
  }
};

bool is_collision(const Trajectory &a, const Trajectory &b)
{
  size_t nSteps = a.poses.size();
  if (nSteps != b.poses.size()) {
    throw std::invalid_argument("Trajectories must have same number of time "
                                "steps.");
  }
  for (size_t i = 0; i < nSteps; ++i) {
    Pose diff = a.poses[i] - b.poses[i];
    double d = diff.distance();
    if (d < 0.005) {
      return true;
    }
  }
  return false;
}

auto word_counting(const std::string &corpusPath, const std::string &queryPath)
    -> void {
  
  size_t nTrials = 1000;
  size_t nAgents = 100;
  size_t nSteps = 10;
  auto ego = Trajectory(-1.0, 0.0, nSteps);
  
  size_t nCollisions = 0;
  for (size_t i = 0; i < nTrials; ++i) {
    bool isCollision = false;
    for (size_t j = 0; j < nAgents; ++j) {
      auto agent = Trajectory(0.0, 0.0, nSteps);
      if (is_collision(ego, agent)) {
        isCollision = true;
        break;
      }
    }
    if (isCollision) {++nCollisions;}
  }
  double odds = static_cast<double>(nCollisions) / static_cast<double>
                (nTrials);
  std::cout << "odds= " << odds << std::endl;
}
} // namespace word_counting
