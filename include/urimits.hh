#ifndef URIMITS_HH
#define URIMITS_HH

#include <algorithm>
#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <vector>

#include "dv_msgs/CarState.h"
#include "dv_msgs/ConeArray.h"
#include "dv_msgs/ConeArrayOrdered.h"
#include "path.hh"
#include "visualization_msgs/Marker.h"

using namespace std;

class Urimits {
 private:
  ////////////////
  // STRUCTURES //
  ////////////////
  struct Pos {
    float x, y;
    Pos(const float &_x = 0.0f, const float &_y = 0.0f) {
      x = _x;
      y = _y;
    }
    static float distSq(const Pos &c1, const Pos &c2) {
      return (c1.x - c2.x) * (c1.x - c2.x) + (c1.y - c2.y) * (c1.y - c2.y);
    }
    Pos operator-(const Pos &p) const {
      return Pos(x - p.x, y - p.y);
    }
    // Compute the angle between two vectors
    static float angle(const Pos &ab, const Pos &cb) {
      float dot = ab.x * cb.x + ab.y * cb.y;
      float det = ab.x * cb.y - ab.y * cb.x;
      return atan2(det, dot);
    }
    static float realAngleDifference(float a1, float a2) {
      if (a1 * a2 < 0)
        return (2 * M_PI - abs(a1) - abs(a2));
      else
        return abs(a1 - a2);
    }

    inline static bool ccw(const Pos &A, const Pos &B, const Pos &C) {
      return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
    }
    // Return true if line segments AB and CD intersect
    static bool intersect(const Pos &A, const Pos &B, const Pos &C, const Pos &D) {
      return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D);
    }

    // Useful to print a Pos
    friend ostream &operator<<(ostream &os, Pos const &pos) {
      return os << "Pos(" << pos.x << ", " << pos.y << ")";
    }
  };
  struct State {
    Pos pos;
    Pos v;
    float angle;
    Path path;
    State(Pos pos, Pos v, float angle) : pos(pos), v(v), angle(angle) {}
  };

  ////////////////
  // ATTRIBUTES //
  ////////////////
  vector<Pos> allCones;
  dv_msgs::ConeArray data;
  Path leftPath, rightPath;
  set<int> indexesToExclude;

  /////////////////////
  // PRIVATE METHODS //
  /////////////////////
  void reset();
  void computeTrace(Path &, const bool &, bool);
  void computeTraceWithCorrection(Path &output, Path &calculatedPath, const bool &);
  int nextConeIndex(const State &actState, const bool &firstLeft, const bool &firstRight) const;
  inline bool isLoopClosed(const Path &path) const;
  void updateState(State &stateToUpdate, const int &nextConeIndex, const bool &isFirst) const;
  bool segmentIntersectsWithPath(const int &c1, const int &c2, const Path &path) const;
  bool pathIntersectsWithItself(const Path &path) const;
  State stateFromPath(const Path &path) const;
  float getHeuristic(const Pos &, const State &, const bool &, const bool &) const;
  list<int> getPossibleCones(const State &) const;
  dv_msgs::ConeArrayOrdered *getTLs() const;
  inline bool stopCondition(const int &nextPossibleIndex, const State &state) const;
  bool anyIntersection() const;

 public:
  // CONSTRUCTOR
  Urimits();

  // PARAMS
  float max_radius_to_next_cone;
  int max_num_cones_to_consider;
  float first_pseudoPosition_offset;
  float dist_ponderation, min_angle_between_3_cones;
  int min_trace_loop_length;

  ////////////////////
  // PUBLIC METHODS //
  ////////////////////
  void run(const dv_msgs::ConeArray::ConstPtr &, const bool &);
  void publishData(const ros::Publisher &) const;
};

#endif