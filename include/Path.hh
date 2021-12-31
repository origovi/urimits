#ifndef PATH_HH
#define PATH_HH

#include <ros/ros.h>

#include <iostream>
#include <memory>

using namespace std;

class Path {
 private:
  struct Connection {
    int coneIndex;
    float angle;
    shared_ptr<Connection> before;

    Connection(const int &coneIndex, shared_ptr<Connection> before)
        : coneIndex(coneIndex), before(before), angle(M_PI) {}
    bool containsCone(const int &_coneIndex) {
      return coneIndex == _coneIndex || (before ? before->containsCone(_coneIndex) : false);
    }

    // Useful to print a Path
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
  Path(shared_ptr<Connection> p)
      : p(p) {}

 public:
  Path()
      : p(nullptr) {}

  Path(const int &x) {
    p = make_shared<Connection>(x, nullptr);
  }

  void addCone(const int &coneIndex, const float &angle) {
    if (!empty()) p->angle = angle;
    p = make_shared<Connection>(coneIndex, p);
  }

  bool empty() const {
    return not p;
  }

  int size() const {
    int res = 0;
    if (not empty()) res = 1 + Path(p->before).size();
    return res;
  }

  Path before() const {
    ROS_ASSERT(not empty());
    return Path(p->before);
  }

  Path first() const {
    ROS_ASSERT(not empty());
    shared_ptr<Connection> lastNotEmpty = p;
    while (lastNotEmpty->before != nullptr) {
      lastNotEmpty = lastNotEmpty->before;
    }
    return Path(lastNotEmpty);
  }

  // CANT RUN IF NOT SURE THE CONE IS IN THIS
  Path getConeIndexPath(const int &index) {
    if (coneIndex() == index)
      return *this;
    else
      return Path(p->before).getConeIndexPath(index);
  }

  const int &coneIndex() const {
    ROS_ASSERT(not empty());
    return p->coneIndex;
  }

  const float &angle() const {
    ROS_ASSERT(not empty());
    return p->angle;
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

  // Useful to print a Path
  friend ostream &operator<<(ostream &os, const Path &path) {
    os << "Path(";
    if (!path.empty()) os << *(path.p);
    return os << ")";
  }
};

#endif