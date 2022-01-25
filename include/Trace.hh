#ifndef TRACE_HH
#define TRACE_HH

#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <list>

using namespace std;

class Trace {
 private:
  struct Connection {
    int coneIndex;
    float angle;
    float heuristic;
    shared_ptr<Connection> before;

    Connection(const int &coneIndex, const float &heuristic, shared_ptr<Connection> before)
        : coneIndex(coneIndex), before(before), heuristic(heuristic), angle(M_PI) {}
    bool containsCone(const int &_coneIndex) {
      return coneIndex == _coneIndex || (before ? before->containsCone(_coneIndex) : false);
    }

    // Useful to print a Trace
    friend ostream &operator<<(ostream &os, const Connection &conn) {
      if (conn.before != nullptr) {
        os << *(conn.before);
      }
      return os << conn.coneIndex << " -> ";
    }
  };

  // ATTRIBUTE (only)
  shared_ptr<Connection> p;

  // CONSTRUCTORS
  Trace(shared_ptr<Connection> p)
      : p(p) {}

  // METHODS

 public:
  Trace()
      : p(nullptr) {}

  void addCone(const int &coneIndex, const float &heuristic, const float &angle) {
    if (!empty()) p->angle = angle;
    p = make_shared<Connection>(coneIndex, heuristic, p);
  }

  bool empty() const {
    return not p;
  }

  int size() const {
    int res = 0;
    if (not empty()) res = 1 + Trace(p->before).size();
    return res;
  }

  Trace before() const {
    ROS_ASSERT(not empty());
    return Trace(p->before);
  }

  Trace first() const {
    ROS_ASSERT(not empty());
    shared_ptr<Connection> lastNotEmpty = p;
    while (lastNotEmpty->before != nullptr) {
      lastNotEmpty = lastNotEmpty->before;
    }
    return Trace(lastNotEmpty);
  }

  // CANT RUN IF NOT SURE THE CONE IS IN *THIS
  Trace getConeIndexTrace(const int &index) {
    if (index == -1 or coneIndex() == index)
      return *this;
    else
      return Trace(p->before).getConeIndexTrace(index);
  }

  // CANT RUN IF NOT SURE THE CONE IS IN *THIS
  Trace getAfterConeIndexTrace(const int &index) {
    if (index == -1) return Trace();
    Trace res;
    list<shared_ptr<Connection>> indexs;
    shared_ptr<Connection> aux = p;
    while (aux->coneIndex != index) {
      indexs.push_front(aux);
      aux = aux->before;
    }
    indexs.push_front(aux);
    for (auto it = indexs.begin(); it != indexs.end(); ++it) {
      res.addCone((*it)->coneIndex, (*it)->heuristic, (*it)->angle);
    }
    return res;
  }

  int coneIndex() const {
    if (empty()) return -1;
    return p->coneIndex;
  }

  const float &angle() const {
    ROS_ASSERT(not empty());
    return p->angle;
  }

  const float &heuristic() const {
    ROS_ASSERT(not empty());
    return p->heuristic;
  }

  float sumAngleDiff() const {
    if (empty()) return 0.0;
    return 1-abs(angle() / M_PI)+before().sumAngleDiff();
  }

  float sumHeuristic() const {
    if (empty()) return 0.0;
    return heuristic() + before().sumHeuristic();
  }

  float avgHeuristic() const {
    if (empty()) return 1e5;
    return sumHeuristic()/size();
  }

  int numSignChanges(bool wasLastPositive) const {
    if (empty()) return 0;
    bool isActualPositive = p->angle >= 0;
    if (isActualPositive != wasLastPositive) return 1+before().numSignChanges(isActualPositive);
    else return before().numSignChanges(isActualPositive);
  }

  bool containsCone(const int &coneIndex) const {
    if (empty())
      return false;
    else
      return p->containsCone(coneIndex);
  }

  void clear() {
    p = nullptr;
  }

  // Useful to print a Trace
  friend ostream &operator<<(ostream &os, const Trace &trace) {
    os << "Trace(";
    if (!trace.empty()) os << *(trace.p);
    return os << ")";
  }
};

#endif